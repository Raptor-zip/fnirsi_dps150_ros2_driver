#include "fnirsi_dps150_driver/dps150_client.hpp"

#include <array>
#include <stdexcept>
#include <utility>

namespace fnirsi_dps150_driver
{

Dps150Client::~Dps150Client()
{
  disconnect();
}

void Dps150Client::set_state_callback(StateCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  state_callback_ = std::move(callback);
}

void Dps150Client::set_error_callback(ErrorCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  error_callback_ = std::move(callback);
}

void Dps150Client::set_command_delay(std::chrono::milliseconds delay)
{
  command_delay_ = delay;
}

void Dps150Client::set_read_timeout(std::chrono::milliseconds timeout)
{
  read_timeout_ = timeout;
}

void Dps150Client::connect(const std::string & device, int baud_rate)
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (connected_) {
    throw std::runtime_error("DPS-150 is already connected");
  }

  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  parser_.reset();
  state_apply_errors_ = 0;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    state_ = DeviceState();
  }

  serial_.open(device, baud_rate);

  try {
    running_ = true;
    connected_ = true;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      state_.connected = true;
    }
    publish_state();

    read_thread_ = std::thread(&Dps150Client::read_loop, this);

    send_byte(protocol::kCommandSession, 0x00, 0x01);
    send_byte(
      protocol::kCommandBaud,
      0x00,
      static_cast<std::uint8_t>(protocol::baud_rate_code(baud_rate)));
    send_byte(protocol::kCommandGet, protocol::kRegModelName, 0x00);
    send_byte(protocol::kCommandGet, protocol::kRegHardwareVersion, 0x00);
    send_byte(protocol::kCommandGet, protocol::kRegFirmwareVersion, 0x00);
    request_all();
  } catch (...) {
    running_ = false;
    connected_ = false;
    serial_.close();
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      state_.connected = false;
    }
    publish_state();
    throw;
  }
}

void Dps150Client::disconnect() noexcept
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  if (!connected_ && !serial_.is_open()) {
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    return;
  }

  if (connected_) {
    try {
      send_byte(protocol::kCommandSession, 0x00, 0x00);
    } catch (...) {
      // Closing the file descriptor below is still the correct recovery path.
    }
  }

  running_ = false;
  connected_ = false;
  serial_.close();

  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  mark_disconnected();
}

bool Dps150Client::is_connected() const noexcept
{
  return connected_;
}

DeviceState Dps150Client::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

void Dps150Client::request_all()
{
  send_byte(protocol::kCommandGet, protocol::kRegAll, 0x00);
}

void Dps150Client::set_voltage(float volts)
{
  send_float(protocol::kCommandSet, protocol::kRegVoltageSet, volts);
}

void Dps150Client::set_current(float amps)
{
  send_float(protocol::kCommandSet, protocol::kRegCurrentSet, amps);
}

void Dps150Client::set_output_enabled(bool enabled)
{
  send_byte(protocol::kCommandSet, protocol::kRegOutputEnable, enabled ? 1 : 0);
}

void Dps150Client::set_metering_enabled(bool enabled)
{
  send_byte(protocol::kCommandSet, protocol::kRegMeteringEnable, enabled ? 1 : 0);
}

void Dps150Client::set_brightness(std::uint8_t value)
{
  send_byte(protocol::kCommandSet, protocol::kRegBrightness, value);
}

void Dps150Client::set_volume(std::uint8_t value)
{
  send_byte(protocol::kCommandSet, protocol::kRegVolume, value);
}

void Dps150Client::set_preset(std::uint8_t index, float volts, float amps, bool apply)
{
  send_float(protocol::kCommandSet, protocol::preset_voltage_register(index), volts);
  send_float(protocol::kCommandSet, protocol::preset_current_register(index), amps);

  if (apply) {
    set_voltage(volts);
    set_current(amps);
  }
}

void Dps150Client::select_preset(std::uint8_t index)
{
  const auto snapshot = state();
  if (index < 1 || index > snapshot.preset_voltages.size()) {
    throw std::out_of_range("preset index must be in the range 1..6");
  }

  const auto offset = static_cast<std::size_t>(index - 1);
  set_voltage(snapshot.preset_voltages[offset]);
  set_current(snapshot.preset_currents[offset]);
}

void Dps150Client::set_protection(std::uint8_t kind, float value)
{
  std::uint8_t reg = 0;
  switch (kind) {
    case 1:
      reg = protocol::kRegOvp;
      break;
    case 2:
      reg = protocol::kRegOcp;
      break;
    case 3:
      reg = protocol::kRegOpp;
      break;
    case 4:
      reg = protocol::kRegOtp;
      break;
    case 5:
      reg = protocol::kRegLvp;
      break;
    default:
      throw std::out_of_range("protection kind must be one of OVP, OCP, OPP, OTP, LVP");
  }

  send_float(protocol::kCommandSet, reg, value);
}

void Dps150Client::send_command(
  std::uint8_t command,
  std::uint8_t reg,
  const std::vector<std::uint8_t> & data)
{
  ensure_connected();
  const auto frame = protocol::encode(protocol::kHeaderTx, command, reg, data);

  std::lock_guard<std::mutex> lock(command_mutex_);
  serial_.write_all(frame);
  std::this_thread::sleep_for(command_delay_);
}

void Dps150Client::send_byte(std::uint8_t command, std::uint8_t reg, std::uint8_t value)
{
  send_command(command, reg, std::vector<std::uint8_t>{value});
}

void Dps150Client::send_float(std::uint8_t command, std::uint8_t reg, float value)
{
  send_command(command, reg, protocol::float_to_little_endian(value));
}

void Dps150Client::read_loop()
{
  std::array<std::uint8_t, 1024> buffer {};

  while (running_) {
    try {
      const auto size = serial_.read_some(buffer.data(), buffer.size(), read_timeout_);
      if (size == 0) {
        continue;
      }

      auto frames = parser_.feed(buffer.data(), size);
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        state_.checksum_errors = parser_.stats().checksum_errors;
        state_.parse_errors = parser_.stats().parse_errors + state_apply_errors_;
      }

      for (const auto & frame : frames) {
        handle_frame(frame);
      }
    } catch (const std::exception & error) {
      if (running_) {
        report_error(error.what());
        running_ = false;
        connected_ = false;
        serial_.close();
        mark_disconnected();
      }
      return;
    }
  }
}

void Dps150Client::handle_frame(const protocol::Frame & frame)
{
  bool updated = false;
  std::string error_message;

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    ++state_.frames_received;
    state_.checksum_errors = parser_.stats().checksum_errors;
    state_.parse_errors = parser_.stats().parse_errors + state_apply_errors_;

    try {
      updated = apply_frame_to_state(frame, state_);
    } catch (const std::exception & error) {
      ++state_apply_errors_;
      state_.parse_errors = parser_.stats().parse_errors + state_apply_errors_;
      error_message = error.what();
    }
  }

  if (!error_message.empty()) {
    report_error(error_message);
  }

  if (updated) {
    publish_state();
  }
}

void Dps150Client::mark_disconnected()
{
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    state_.connected = false;
  }
  publish_state();
}

void Dps150Client::publish_state()
{
  StateCallback callback;
  DeviceState snapshot;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    snapshot = state_;
  }
  {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    callback = state_callback_;
  }

  if (callback) {
    callback(snapshot);
  }
}

void Dps150Client::report_error(const std::string & message)
{
  ErrorCallback callback;
  {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    callback = error_callback_;
  }

  if (callback) {
    callback(message);
  }
}

void Dps150Client::ensure_connected() const
{
  if (!connected_) {
    throw std::runtime_error("DPS-150 is not connected");
  }
}

}  // namespace fnirsi_dps150_driver
