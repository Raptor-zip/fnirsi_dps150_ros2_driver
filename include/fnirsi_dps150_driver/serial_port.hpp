#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace fnirsi_dps150_driver
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void open(const std::string & device, int baud_rate);
  void close() noexcept;
  bool is_open() const noexcept;

  std::size_t read_some(
    std::uint8_t * buffer,
    std::size_t size,
    std::chrono::milliseconds timeout);

  void write_all(const std::vector<std::uint8_t> & data);

private:
  int fd_{-1};
  mutable std::mutex mutex_;
};

}  // namespace fnirsi_dps150_driver
