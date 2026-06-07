#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "fnirsi_dps150_driver/device_state.hpp"
#include "fnirsi_dps150_driver/protocol.hpp"
#include "fnirsi_dps150_driver/serial_port.hpp"

namespace fnirsi_dps150_driver
{

class Dps150Client
{
public:
  using StateCallback = std::function<void(const DeviceState &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  Dps150Client() = default;
  ~Dps150Client();

  Dps150Client(const Dps150Client &) = delete;
  Dps150Client & operator=(const Dps150Client &) = delete;

  void set_state_callback(StateCallback callback);
  void set_error_callback(ErrorCallback callback);
  void set_command_delay(std::chrono::milliseconds delay);
  void set_read_timeout(std::chrono::milliseconds timeout);

  void connect(const std::string & device, int baud_rate);
  void disconnect() noexcept;
  bool is_connected() const noexcept;

  DeviceState state() const;

  void request_all();
  void set_voltage(float volts);
  void set_current(float amps);
  void set_output_enabled(bool enabled);
  void set_metering_enabled(bool enabled);
  void set_brightness(std::uint8_t value);
  void set_volume(std::uint8_t value);
  void set_preset(std::uint8_t index, float volts, float amps, bool apply);
  void select_preset(std::uint8_t index);
  void set_protection(std::uint8_t kind, float value);

private:
  void send_command(
    std::uint8_t command,
    std::uint8_t reg,
    const std::vector<std::uint8_t> & data);
  void send_byte(std::uint8_t command, std::uint8_t reg, std::uint8_t value);
  void send_float(std::uint8_t command, std::uint8_t reg, float value);
  void read_loop();
  void handle_frame(const protocol::Frame & frame);
  void mark_disconnected();
  void publish_state();
  void report_error(const std::string & message);
  void ensure_connected() const;

  SerialPort serial_;
  mutable std::mutex state_mutex_;
  std::mutex callback_mutex_;
  std::mutex command_mutex_;
  std::mutex lifecycle_mutex_;

  DeviceState state_;
  protocol::FrameParser parser_;
  StateCallback state_callback_;
  ErrorCallback error_callback_;

  std::thread read_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool connected_{false};
  std::uint64_t state_apply_errors_{0};
  std::chrono::milliseconds command_delay_{50};
  std::chrono::milliseconds read_timeout_{100};
};

}  // namespace fnirsi_dps150_driver
