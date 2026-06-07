#include "fnirsi_dps150_driver/dps150_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace fnirsi_dps150_driver
{
namespace
{

bool valid_positive_limit(float value)
{
  return std::isfinite(value) && value > 0.0F;
}

void require_finite(float value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

}  // namespace

Dps150Node::Dps150Node(const rclcpp::NodeOptions & options)
: Node("dps150", options)
{
  port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
  baud_rate_ = declare_parameter<int>("baud_rate", 115200);
  connect_on_start_ = declare_parameter<bool>("connect_on_start", true);
  auto_refresh_after_write_ = declare_parameter<bool>("auto_refresh_after_write", true);
  auto_reconnect_ = declare_parameter<bool>("auto_reconnect", true);
  enforce_setpoint_limits_ = declare_parameter<bool>("enforce_setpoint_limits", true);
  allow_output_enable_during_protection_ =
    declare_parameter<bool>("allow_output_enable_during_protection", false);

  const auto command_delay_ms = declare_parameter<int>("command_delay_ms", 50);
  const auto read_timeout_ms = declare_parameter<int>("read_timeout_ms", 100);
  const auto publish_period_ms = declare_parameter<int>("state_publish_period_ms", 1000);
  reconnect_period_ms_ = declare_parameter<int>("reconnect_period_ms", 2000);

  manual_disconnect_ = !connect_on_start_;

  client_.set_command_delay(std::chrono::milliseconds(command_delay_ms));
  client_.set_read_timeout(std::chrono::milliseconds(read_timeout_ms));

  state_pub_ = create_publisher<msg::Dps150State>("~/state", rclcpp::SensorDataQoS());

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("FNIRSI DPS-150");
  diagnostics_->add("DPS-150", this, &Dps150Node::update_diagnostics);

  client_.set_state_callback(
    [this](const DeviceState & state) {
      last_state_time_ = now();
      publish_state(state);
    });
  client_.set_error_callback(
    [this](const std::string & message) {
      last_error_ = message;
      RCLCPP_WARN(get_logger(), "DPS-150 driver error: %s", message.c_str());
    });

  voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
    "~/set_voltage",
    10,
    [this](const std_msgs::msg::Float32::SharedPtr message) {
      run_command("set_voltage subscription", [this, message]() {
        checked_set_voltage(message->data);
        refresh_after_write(true);
      });
    });

  current_sub_ = create_subscription<std_msgs::msg::Float32>(
    "~/set_current",
    10,
    [this](const std_msgs::msg::Float32::SharedPtr message) {
      run_command("set_current subscription", [this, message]() {
        checked_set_current(message->data);
        refresh_after_write(true);
      });
    });

  output_sub_ = create_subscription<std_msgs::msg::Bool>(
    "~/output_enable",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      run_command("output_enable subscription", [this, message]() {
        checked_set_output_enabled(message->data);
        refresh_after_write(true);
      });
    });

  connect_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/connect",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
      try {
        manual_disconnect_ = false;
        connect_device();
        response->success = true;
        response->message = "connected";
      } catch (const std::exception & error) {
        response->success = false;
        response->message = error.what();
        last_error_ = error.what();
      }
    });

  disconnect_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/disconnect",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
      disconnect_device();
      response->success = true;
      response->message = "disconnected";
    });

  refresh_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/refresh",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
      try {
        client_.request_all();
        response->success = true;
        response->message = "refresh requested";
      } catch (const std::exception & error) {
        response->success = false;
        response->message = error.what();
        last_error_ = error.what();
      }
    });

  enable_output_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_output",
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      const std_srvs::srv::SetBool::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        checked_set_output_enabled(request->data);
        refresh_after_write(true);
        return request->data ? "output enabled" : "output disabled";
      });
    });

  enable_metering_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_metering",
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      const std_srvs::srv::SetBool::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        client_.set_metering_enabled(request->data);
        refresh_after_write(true);
        return request->data ? "metering enabled" : "metering disabled";
      });
    });

  set_voltage_srv_ = create_service<srv::SetFloat32>(
    "~/set_voltage",
    [this](
      const srv::SetFloat32::Request::SharedPtr request,
      const srv::SetFloat32::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        checked_set_voltage(request->value);
        refresh_after_write(request->refresh);
        return "voltage setpoint written";
      });
    });

  set_current_srv_ = create_service<srv::SetFloat32>(
    "~/set_current",
    [this](
      const srv::SetFloat32::Request::SharedPtr request,
      const srv::SetFloat32::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        checked_set_current(request->value);
        refresh_after_write(request->refresh);
        return "current limit written";
      });
    });

  set_brightness_srv_ = create_service<srv::SetUInt8>(
    "~/set_brightness",
    [this](
      const srv::SetUInt8::Request::SharedPtr request,
      const srv::SetUInt8::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        client_.set_brightness(request->value);
        refresh_after_write(request->refresh);
        return "brightness written";
      });
    });

  set_volume_srv_ = create_service<srv::SetUInt8>(
    "~/set_volume",
    [this](
      const srv::SetUInt8::Request::SharedPtr request,
      const srv::SetUInt8::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        client_.set_volume(request->value);
        refresh_after_write(request->refresh);
        return "volume written";
      });
    });

  set_preset_srv_ = create_service<srv::SetPreset>(
    "~/set_preset",
    [this](
      const srv::SetPreset::Request::SharedPtr request,
      const srv::SetPreset::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        checked_set_preset(request->index, request->voltage, request->current, request->apply);
        refresh_after_write(request->refresh);
        return "preset written";
      });
    });

  select_preset_srv_ = create_service<srv::SelectPreset>(
    "~/select_preset",
    [this](
      const srv::SelectPreset::Request::SharedPtr request,
      const srv::SelectPreset::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        checked_select_preset(request->index);
        refresh_after_write(request->refresh);
        return "preset applied";
      });
    });

  set_protection_srv_ = create_service<srv::SetProtection>(
    "~/set_protection",
    [this](
      const srv::SetProtection::Request::SharedPtr request,
      const srv::SetProtection::Response::SharedPtr response) {
      service_command(response->success, response->message, [this, request]() {
        require_finite(request->value, "protection value");
        client_.set_protection(request->kind, request->value);
        refresh_after_write(request->refresh);
        return "protection threshold written";
      });
    });

  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms),
    [this]() {
      publish_state(client_.state());
    });

  reconnect_timer_ = create_wall_timer(
    std::chrono::milliseconds(std::max(100, reconnect_period_ms_)),
    [this]() {
      reconnect_tick();
    });

  if (connect_on_start_) {
    try {
      connect_device();
    } catch (const std::exception & error) {
      last_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "Failed to connect to DPS-150 on %s: %s", port_.c_str(), error.what());
    }
  }
}

Dps150Node::~Dps150Node()
{
  client_.disconnect();
}

void Dps150Node::connect_device()
{
  if (client_.is_connected()) {
    return;
  }
  client_.connect(port_, baud_rate_);
  last_error_.clear();
  RCLCPP_INFO(get_logger(), "Connected to DPS-150 on %s", port_.c_str());
}

void Dps150Node::disconnect_device()
{
  manual_disconnect_ = true;
  client_.disconnect();
}

void Dps150Node::refresh_after_write(bool request_refresh)
{
  if (auto_refresh_after_write_ || request_refresh) {
    client_.request_all();
  }
}

void Dps150Node::reconnect_tick()
{
  if (!auto_reconnect_ || manual_disconnect_ || client_.is_connected()) {
    return;
  }

  ++reconnect_attempts_;
  try {
    connect_device();
    ++successful_reconnects_;
  } catch (const std::exception & error) {
    last_error_ = error.what();
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,
      "DPS-150 reconnect attempt failed on %s: %s",
      port_.c_str(),
      error.what());
  }
}

void Dps150Node::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto state = client_.state();

  if (!state.connected) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Disconnected");
  } else if (state.checksum_errors > 0 || state.parse_errors > 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Serial parse errors observed");
  } else if (state.protection_code != msg::Dps150State::PROTECTION_OK) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Device protection state is active");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Connected");
  }

  status.add("connected", state.connected);
  status.add("port", port_);
  status.add("model_name", state.model_name);
  status.add("hardware_version", state.hardware_version);
  status.add("firmware_version", state.firmware_version);
  status.add("input_voltage", state.input_voltage);
  status.add("set_voltage", state.set_voltage);
  status.add("set_current", state.set_current);
  status.add("output_voltage", state.output_voltage);
  status.add("output_current", state.output_current);
  status.add("output_power", state.output_power);
  status.add("temperature", state.temperature);
  status.add("output_enabled", state.output_enabled);
  status.add("protection", state.protection);
  status.add("mode", state.mode);
  status.add("frames_received", state.frames_received);
  status.add("checksum_errors", state.checksum_errors);
  status.add("parse_errors", state.parse_errors);
  if (last_state_time_.nanoseconds() > 0) {
    status.add("last_state_age_sec", (now() - last_state_time_).seconds());
  }
  status.add("auto_reconnect", auto_reconnect_);
  status.add("reconnect_attempts", reconnect_attempts_);
  status.add("successful_reconnects", successful_reconnects_);
  status.add("last_error", last_error_);
}

void Dps150Node::checked_set_voltage(float volts)
{
  require_finite(volts, "voltage");
  const auto state = client_.state();
  validate_voltage(volts, state);
  validate_power(volts, state.set_current, state);
  client_.set_voltage(volts);
}

void Dps150Node::checked_set_current(float amps)
{
  require_finite(amps, "current");
  const auto state = client_.state();
  validate_current(amps, state);
  validate_power(state.set_voltage, amps, state);
  client_.set_current(amps);
}

void Dps150Node::checked_set_output_enabled(bool enabled)
{
  if (enabled && !allow_output_enable_during_protection_) {
    const auto state = client_.state();
    if (state.protection_code != msg::Dps150State::PROTECTION_OK) {
      throw std::runtime_error("refusing to enable output while protection state is active: " + state.protection);
    }
    validate_voltage(state.set_voltage, state);
    validate_current(state.set_current, state);
    validate_power(state.set_voltage, state.set_current, state);
  }

  client_.set_output_enabled(enabled);
}

void Dps150Node::checked_set_preset(std::uint8_t index, float volts, float amps, bool apply)
{
  require_finite(volts, "preset voltage");
  require_finite(amps, "preset current");
  const auto state = client_.state();
  validate_voltage(volts, state);
  validate_current(amps, state);
  validate_power(volts, amps, state);
  client_.set_preset(index, volts, amps, apply);
}

void Dps150Node::checked_select_preset(std::uint8_t index)
{
  const auto state = client_.state();
  if (index < 1 || index > state.preset_voltages.size()) {
    throw std::out_of_range("preset index must be in the range 1..6");
  }

  const auto offset = static_cast<std::size_t>(index - 1);
  validate_voltage(state.preset_voltages[offset], state);
  validate_current(state.preset_currents[offset], state);
  validate_power(state.preset_voltages[offset], state.preset_currents[offset], state);
  client_.select_preset(index);
}

void Dps150Node::validate_voltage(float volts, const DeviceState & state) const
{
  if (volts < 0.0F) {
    throw std::out_of_range("voltage must be non-negative");
  }
  if (!enforce_setpoint_limits_) {
    return;
  }
  if (valid_positive_limit(state.upper_limit_voltage) && volts > state.upper_limit_voltage) {
    throw std::out_of_range("voltage exceeds device upper limit");
  }
  if (valid_positive_limit(state.over_voltage_protection) && volts > state.over_voltage_protection) {
    throw std::out_of_range("voltage exceeds OVP threshold");
  }
}

void Dps150Node::validate_current(float amps, const DeviceState & state) const
{
  if (amps < 0.0F) {
    throw std::out_of_range("current must be non-negative");
  }
  if (!enforce_setpoint_limits_) {
    return;
  }
  if (valid_positive_limit(state.upper_limit_current) && amps > state.upper_limit_current) {
    throw std::out_of_range("current exceeds device upper limit");
  }
  if (valid_positive_limit(state.over_current_protection) && amps > state.over_current_protection) {
    throw std::out_of_range("current exceeds OCP threshold");
  }
}

void Dps150Node::validate_power(float volts, float amps, const DeviceState & state) const
{
  if (!enforce_setpoint_limits_) {
    return;
  }
  if (!std::isfinite(volts) || !std::isfinite(amps) || !valid_positive_limit(state.over_power_protection)) {
    return;
  }
  if ((volts * amps) > state.over_power_protection) {
    throw std::out_of_range("voltage/current setpoint exceeds OPP threshold");
  }
}

void Dps150Node::publish_state(const DeviceState & state)
{
  state_pub_->publish(to_msg(state));
}

msg::Dps150State Dps150Node::to_msg(const DeviceState & state)
{
  msg::Dps150State message;
  message.stamp = now();
  message.connected = state.connected;
  message.input_voltage = state.input_voltage;
  message.set_voltage = state.set_voltage;
  message.set_current = state.set_current;
  message.output_voltage = state.output_voltage;
  message.output_current = state.output_current;
  message.output_power = state.output_power;
  message.temperature = state.temperature;
  message.preset_voltages = state.preset_voltages;
  message.preset_currents = state.preset_currents;
  message.over_voltage_protection = state.over_voltage_protection;
  message.over_current_protection = state.over_current_protection;
  message.over_power_protection = state.over_power_protection;
  message.over_temperature_protection = state.over_temperature_protection;
  message.low_voltage_protection = state.low_voltage_protection;
  message.brightness = state.brightness;
  message.volume = state.volume;
  message.metering_enabled = state.metering_enabled;
  message.output_capacity = state.output_capacity;
  message.output_energy = state.output_energy;
  message.output_enabled = state.output_enabled;
  message.protection_code = state.protection_code;
  message.protection = state.protection;
  message.mode_code = state.mode_code;
  message.mode = state.mode;
  message.upper_limit_voltage = state.upper_limit_voltage;
  message.upper_limit_current = state.upper_limit_current;
  message.model_name = state.model_name;
  message.hardware_version = state.hardware_version;
  message.firmware_version = state.firmware_version;
  message.frames_received = state.frames_received;
  message.checksum_errors = state.checksum_errors;
  message.parse_errors = state.parse_errors;
  return message;
}

}  // namespace fnirsi_dps150_driver
