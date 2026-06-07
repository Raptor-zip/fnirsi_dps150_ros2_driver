#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cmath>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "fnirsi_dps150_driver/dps150_node.hpp"
#include "fnirsi_dps150_driver/msg/dps150_state.hpp"
#include "fnirsi_dps150_driver/protocol.hpp"
#include "fnirsi_dps150_driver/srv/set_float32.hpp"

namespace
{

using namespace std::chrono_literals;
namespace protocol = fnirsi_dps150_driver::protocol;

class PseudoTerminal
{
public:
  PseudoTerminal()
  {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd_ < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    if (grantpt(master_fd_) != 0 || unlockpt(master_fd_) != 0) {
      throw std::runtime_error(std::strerror(errno));
    }

    char * name = ptsname(master_fd_);
    if (name == nullptr) {
      throw std::runtime_error(std::strerror(errno));
    }
    slave_path_ = name;

    termios tty {};
    if (tcgetattr(master_fd_, &tty) == 0) {
      cfmakeraw(&tty);
      tcsetattr(master_fd_, TCSANOW, &tty);
    }
  }

  ~PseudoTerminal()
  {
    if (master_fd_ >= 0) {
      close(master_fd_);
    }
  }

  PseudoTerminal(const PseudoTerminal &) = delete;
  PseudoTerminal & operator=(const PseudoTerminal &) = delete;

  int master_fd() const { return master_fd_; }
  const std::string & slave_path() const { return slave_path_; }

private:
  int master_fd_{-1};
  std::string slave_path_;
};

class MockDps150
{
public:
  explicit MockDps150(int fd)
  : fd_(fd)
  {}

  ~MockDps150()
  {
    stop();
  }

  void start()
  {
    running_ = true;
    thread_ = std::thread(&MockDps150::loop, this);
  }

  void stop()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void loop()
  {
    std::vector<std::uint8_t> buffer;
    while (running_) {
      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(fd_, &read_set);
      timeval tv {};
      tv.tv_sec = 0;
      tv.tv_usec = 10000;

      const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
      if (ready <= 0) {
        continue;
      }

      std::uint8_t chunk[256] {};
      const auto n = read(fd_, chunk, sizeof(chunk));
      if (n <= 0) {
        continue;
      }
      buffer.insert(buffer.end(), chunk, chunk + n);
      parse(buffer);
    }
  }

  void parse(std::vector<std::uint8_t> & buffer)
  {
    while (true) {
      auto header = std::find(buffer.begin(), buffer.end(), protocol::kHeaderTx);
      if (header == buffer.end()) {
        buffer.clear();
        return;
      }
      if (header != buffer.begin()) {
        buffer.erase(buffer.begin(), header);
      }
      if (buffer.size() < 5) {
        return;
      }

      const std::size_t length = buffer[3];
      const std::size_t total = length + 5;
      if (buffer.size() < total) {
        return;
      }

      const auto command = buffer[1];
      const auto reg = buffer[2];
      std::vector<std::uint8_t> data(buffer.begin() + 4, buffer.begin() + 4 + length);
      const auto expected = protocol::checksum(reg, data);
      const auto actual = buffer[total - 1];
      buffer.erase(buffer.begin(), buffer.begin() + total);

      if (expected != actual) {
        continue;
      }
      handle(command, reg, data);
    }
  }

  void handle(std::uint8_t command, std::uint8_t reg, const std::vector<std::uint8_t> & data)
  {
    if (command == protocol::kCommandGet) {
      if (reg == protocol::kRegModelName) {
        send_string(reg, "DPS-150");
      } else if (reg == protocol::kRegHardwareVersion) {
        send_string(reg, "V1.0");
      } else if (reg == protocol::kRegFirmwareVersion) {
        send_string(reg, "V1.1");
      } else if (reg == protocol::kRegAll) {
        send_full_dump();
      }
      return;
    }

    if (command != protocol::kCommandSet) {
      return;
    }

    if (reg == protocol::kRegVoltageSet && data.size() >= 4) {
      set_voltage_ = protocol::little_endian_to_float(data);
    } else if (reg == protocol::kRegCurrentSet && data.size() >= 4) {
      set_current_ = protocol::little_endian_to_float(data);
    } else if (reg == protocol::kRegOutputEnable && !data.empty()) {
      output_enabled_ = data[0] == 1;
    }

    write_frame(protocol::encode(protocol::kHeaderRx, protocol::kCommandGet, reg, data));
  }

  void send_string(std::uint8_t reg, const std::string & value)
  {
    write_frame(
      protocol::encode(
        protocol::kHeaderRx,
        protocol::kCommandGet,
        reg,
        std::vector<std::uint8_t>(value.begin(), value.end())));
  }

  void send_full_dump()
  {
    std::vector<std::uint8_t> data(139, 0);
    auto put_float = [&data](std::size_t offset, float value) {
        const auto bytes = protocol::float_to_little_endian(value);
        data[offset] = bytes[0];
        data[offset + 1] = bytes[1];
        data[offset + 2] = bytes[2];
        data[offset + 3] = bytes[3];
      };

    put_float(0, 20.0F);
    put_float(4, set_voltage_);
    put_float(8, set_current_);
    put_float(12, output_enabled_ ? set_voltage_ : 0.0F);
    put_float(16, output_enabled_ ? set_current_ : 0.0F);
    put_float(20, output_enabled_ ? (set_voltage_ * set_current_) : 0.0F);
    put_float(24, 28.0F);
    for (std::size_t i = 0; i < 6; ++i) {
      put_float(28 + (i * 8), 5.0F);
      put_float(32 + (i * 8), 1.0F);
    }
    put_float(76, 30.0F);
    put_float(80, 5.1F);
    put_float(84, 150.0F);
    put_float(88, 80.0F);
    put_float(92, 5.0F);
    data[96] = 10;
    data[97] = 10;
    data[98] = 1;
    put_float(99, 0.0F);
    put_float(103, 0.0F);
    data[107] = output_enabled_ ? 1 : 0;
    data[108] = 0;
    data[109] = 1;
    put_float(111, 20.0F);
    put_float(115, 5.1F);

    write_frame(protocol::encode(protocol::kHeaderRx, protocol::kCommandGet, protocol::kRegAll, data));
  }

  void write_frame(const std::vector<std::uint8_t> & frame)
  {
    std::size_t written = 0;
    while (written < frame.size()) {
      const auto n = write(fd_, frame.data() + written, frame.size() - written);
      if (n > 0) {
        written += static_cast<std::size_t>(n);
      }
    }
  }

  int fd_{-1};
  std::atomic_bool running_{false};
  std::thread thread_;
  float set_voltage_{5.0F};
  float set_current_{1.0F};
  bool output_enabled_{false};
};

TEST(Dps150NodeIntegration, PublishesStateAndHandlesSetVoltageService)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  PseudoTerminal pty;
  MockDps150 mock(pty.master_fd());
  mock.start();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("port", pty.slave_path()),
      rclcpp::Parameter("connect_on_start", true),
      rclcpp::Parameter("auto_reconnect", false),
      rclcpp::Parameter("command_delay_ms", 1),
      rclcpp::Parameter("read_timeout_ms", 10),
      rclcpp::Parameter("state_publish_period_ms", 100),
    });

  auto driver = std::make_shared<fnirsi_dps150_driver::Dps150Node>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dps150_integration_test_client");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  std::mutex mutex;
  std::condition_variable cv;
  std::optional<fnirsi_dps150_driver::msg::Dps150State> last_state;
  auto subscription = client_node->create_subscription<fnirsi_dps150_driver::msg::Dps150State>(
    "/dps150/state",
    rclcpp::SensorDataQoS(),
    [&](const fnirsi_dps150_driver::msg::Dps150State::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      last_state = *message;
      cv.notify_all();
    });

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(
      cv.wait_for(
        lock,
        5s,
        [&last_state]() {
          return last_state.has_value() && last_state->connected && last_state->model_name == "DPS-150";
        }));
    EXPECT_EQ(last_state->checksum_errors, 0U);
    EXPECT_EQ(last_state->parse_errors, 0U);
  }

  auto set_voltage_client =
    client_node->create_client<fnirsi_dps150_driver::srv::SetFloat32>("/dps150/set_voltage");
  ASSERT_TRUE(set_voltage_client->wait_for_service(2s));

  auto request = std::make_shared<fnirsi_dps150_driver::srv::SetFloat32::Request>();
  request->value = 7.5F;
  request->refresh = true;
  auto response_future = set_voltage_client->async_send_request(request);
  ASSERT_EQ(response_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(response_future.get()->success);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(
      cv.wait_for(
        lock,
        5s,
        [&last_state]() {
          return last_state.has_value() && std::fabs(last_state->set_voltage - 7.5F) < 0.001F;
        }));
    EXPECT_FLOAT_EQ(last_state->set_current, 1.0F);
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  executor.remove_node(client_node);
  executor.remove_node(driver);
  driver.reset();
  client_node.reset();
  mock.stop();
  rclcpp::shutdown();
}

}  // namespace
