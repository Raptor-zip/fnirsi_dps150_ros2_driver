#include "fnirsi_dps150_driver/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace fnirsi_dps150_driver
{
namespace
{

speed_t termios_baud(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      throw std::invalid_argument("unsupported serial baud rate");
  }
}

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & device, int baud_rate)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (fd_ >= 0) {
    throw std::runtime_error("serial port is already open");
  }

  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error(errno_message("failed to open " + device));
  }

  termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    const auto message = errno_message("failed to read serial attributes");
    ::close(fd);
    throw std::runtime_error(message);
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, termios_baud(baud_rate));
  cfsetospeed(&tty, termios_baud(baud_rate));

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    const auto message = errno_message("failed to configure serial port");
    ::close(fd);
    throw std::runtime_error(message);
  }

  if (tcflush(fd, TCIOFLUSH) != 0) {
    const auto message = errno_message("failed to flush serial port");
    ::close(fd);
    throw std::runtime_error(message);
  }

  fd_ = fd;
}

void SerialPort::close() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

std::size_t SerialPort::read_some(
  std::uint8_t * buffer,
  std::size_t size,
  std::chrono::milliseconds timeout)
{
  int fd = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fd = fd_;
  }

  if (fd < 0) {
    throw std::runtime_error("serial port is not open");
  }

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd, &read_set);

  timeval tv {};
  tv.tv_sec = static_cast<long>(timeout.count() / 1000);
  tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

  int ready = 0;
  do {
    ready = select(fd + 1, &read_set, nullptr, nullptr, &tv);
  } while (ready < 0 && errno == EINTR);

  if (ready < 0) {
    throw std::runtime_error(errno_message("serial select failed"));
  }
  if (ready == 0) {
    return 0;
  }

  const auto n = ::read(fd, buffer, size);
  if (n > 0) {
    return static_cast<std::size_t>(n);
  }
  if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
    return 0;
  }

  throw std::runtime_error(errno_message("serial read failed"));
}

void SerialPort::write_all(const std::vector<std::uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw std::runtime_error("serial port is not open");
  }

  std::size_t written = 0;
  while (written < data.size()) {
    const auto n = ::write(fd_, data.data() + written, data.size() - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }
    throw std::runtime_error(errno_message("serial write failed"));
  }

  if (tcdrain(fd_) != 0) {
    throw std::runtime_error(errno_message("serial drain failed"));
  }
}

}  // namespace fnirsi_dps150_driver
