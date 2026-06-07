#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>

#include "fnirsi_dps150_driver/protocol.hpp"

namespace fnirsi_dps150_driver
{

struct DeviceState
{
  DeviceState();

  bool connected{false};

  float input_voltage{nan()};
  float set_voltage{nan()};
  float set_current{nan()};
  float output_voltage{nan()};
  float output_current{nan()};
  float output_power{nan()};
  float temperature{nan()};

  std::array<float, 6> preset_voltages{};
  std::array<float, 6> preset_currents{};

  float over_voltage_protection{nan()};
  float over_current_protection{nan()};
  float over_power_protection{nan()};
  float over_temperature_protection{nan()};
  float low_voltage_protection{nan()};

  std::uint8_t brightness{0};
  std::uint8_t volume{0};
  bool metering_enabled{false};
  float output_capacity{nan()};
  float output_energy{nan()};

  bool output_enabled{false};
  std::uint8_t protection_code{protocol::kRegAll};
  std::string protection{"UNKNOWN"};
  std::uint8_t mode_code{protocol::kRegAll};
  std::string mode{"UNKNOWN"};

  float upper_limit_voltage{nan()};
  float upper_limit_current{nan()};

  std::string model_name;
  std::string hardware_version;
  std::string firmware_version;

  std::uint64_t frames_received{0};
  std::uint64_t checksum_errors{0};
  std::uint64_t parse_errors{0};

  static constexpr float nan()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
};

bool apply_frame_to_state(const protocol::Frame & frame, DeviceState & state);

}  // namespace fnirsi_dps150_driver
