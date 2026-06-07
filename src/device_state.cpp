#include "fnirsi_dps150_driver/device_state.hpp"

#include <stdexcept>

namespace fnirsi_dps150_driver
{
namespace
{

float f32(const protocol::Frame & frame, std::size_t offset = 0)
{
  return protocol::little_endian_to_float(frame.data, offset);
}

bool has_size(const protocol::Frame & frame, std::size_t size)
{
  return frame.data.size() >= size;
}

void apply_full_dump(const protocol::Frame & frame, DeviceState & state)
{
  if (!has_size(frame, 139)) {
    throw std::runtime_error("full state frame is shorter than 139 bytes");
  }

  state.input_voltage = f32(frame, 0);
  state.set_voltage = f32(frame, 4);
  state.set_current = f32(frame, 8);
  state.output_voltage = f32(frame, 12);
  state.output_current = f32(frame, 16);
  state.output_power = f32(frame, 20);
  state.temperature = f32(frame, 24);

  for (std::size_t i = 0; i < 6; ++i) {
    state.preset_voltages[i] = f32(frame, 28 + (i * 8));
    state.preset_currents[i] = f32(frame, 32 + (i * 8));
  }

  state.over_voltage_protection = f32(frame, 76);
  state.over_current_protection = f32(frame, 80);
  state.over_power_protection = f32(frame, 84);
  state.over_temperature_protection = f32(frame, 88);
  state.low_voltage_protection = f32(frame, 92);

  state.brightness = frame.data[96];
  state.volume = frame.data[97];
  state.metering_enabled = frame.data[98] != 0;
  state.output_capacity = f32(frame, 99);
  state.output_energy = f32(frame, 103);

  state.output_enabled = frame.data[107] == 1;
  state.protection_code = frame.data[108];
  state.protection = protocol::protection_name(state.protection_code);
  state.mode_code = frame.data[109];
  state.mode = protocol::mode_name(state.mode_code);

  state.upper_limit_voltage = f32(frame, 111);
  state.upper_limit_current = f32(frame, 115);
}

}  // namespace

DeviceState::DeviceState()
{
  preset_voltages.fill(nan());
  preset_currents.fill(nan());
}

bool apply_frame_to_state(const protocol::Frame & frame, DeviceState & state)
{
  switch (frame.reg) {
    case protocol::kRegInputVoltage:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("input voltage frame is shorter than 4 bytes");
      }
      state.input_voltage = f32(frame);
      return true;
    case protocol::kRegVoltageSet:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("voltage setpoint frame is shorter than 4 bytes");
      }
      state.set_voltage = f32(frame);
      return true;
    case protocol::kRegCurrentSet:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("current setpoint frame is shorter than 4 bytes");
      }
      state.set_current = f32(frame);
      return true;
    case protocol::kRegMeasurement:
      if (!has_size(frame, 12)) {
        throw std::runtime_error("measurement frame is shorter than 12 bytes");
      }
      state.output_voltage = f32(frame, 0);
      state.output_current = f32(frame, 4);
      state.output_power = f32(frame, 8);
      return true;
    case protocol::kRegTemperature:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("temperature frame is shorter than 4 bytes");
      }
      state.temperature = f32(frame);
      return true;
    case protocol::kRegOvp:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("OVP frame is shorter than 4 bytes");
      }
      state.over_voltage_protection = f32(frame);
      return true;
    case protocol::kRegOcp:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("OCP frame is shorter than 4 bytes");
      }
      state.over_current_protection = f32(frame);
      return true;
    case protocol::kRegOpp:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("OPP frame is shorter than 4 bytes");
      }
      state.over_power_protection = f32(frame);
      return true;
    case protocol::kRegOtp:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("OTP frame is shorter than 4 bytes");
      }
      state.over_temperature_protection = f32(frame);
      return true;
    case protocol::kRegLvp:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("LVP frame is shorter than 4 bytes");
      }
      state.low_voltage_protection = f32(frame);
      return true;
    case protocol::kRegBrightness:
      if (frame.data.empty()) {
        throw std::runtime_error("brightness frame is empty");
      }
      state.brightness = frame.data[0];
      return true;
    case protocol::kRegVolume:
      if (frame.data.empty()) {
        throw std::runtime_error("volume frame is empty");
      }
      state.volume = frame.data[0];
      return true;
    case protocol::kRegMeteringEnable:
      if (frame.data.empty()) {
        throw std::runtime_error("metering frame is empty");
      }
      state.metering_enabled = frame.data[0] != 0;
      return true;
    case protocol::kRegOutputCapacity:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("capacity frame is shorter than 4 bytes");
      }
      state.output_capacity = f32(frame);
      return true;
    case protocol::kRegOutputEnergy:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("energy frame is shorter than 4 bytes");
      }
      state.output_energy = f32(frame);
      return true;
    case protocol::kRegOutputEnable:
      if (frame.data.empty()) {
        throw std::runtime_error("output enable frame is empty");
      }
      state.output_enabled = frame.data[0] == 1;
      return true;
    case protocol::kRegProtectionState:
      if (frame.data.empty()) {
        throw std::runtime_error("protection state frame is empty");
      }
      state.protection_code = frame.data[0];
      state.protection = protocol::protection_name(state.protection_code);
      return true;
    case protocol::kRegMode:
      if (frame.data.empty()) {
        throw std::runtime_error("mode frame is empty");
      }
      state.mode_code = frame.data[0];
      state.mode = protocol::mode_name(state.mode_code);
      return true;
    case protocol::kRegModelName:
      state.model_name = protocol::ascii_string(frame.data);
      return true;
    case protocol::kRegHardwareVersion:
      state.hardware_version = protocol::ascii_string(frame.data);
      return true;
    case protocol::kRegFirmwareVersion:
      state.firmware_version = protocol::ascii_string(frame.data);
      return true;
    case protocol::kRegUpperLimitVoltage:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("upper voltage limit frame is shorter than 4 bytes");
      }
      state.upper_limit_voltage = f32(frame);
      return true;
    case protocol::kRegUpperLimitCurrent:
      if (!has_size(frame, 4)) {
        throw std::runtime_error("upper current limit frame is shorter than 4 bytes");
      }
      state.upper_limit_current = f32(frame);
      return true;
    case protocol::kRegAll:
      apply_full_dump(frame, state);
      return true;
    default:
      break;
  }

  if (protocol::is_preset_register(frame.reg)) {
    if (!has_size(frame, 4)) {
      throw std::runtime_error("preset frame is shorter than 4 bytes");
    }
    const auto offset = static_cast<std::uint8_t>(frame.reg - protocol::kRegPreset1Voltage);
    const auto index = static_cast<std::size_t>(offset / 2);
    if ((offset % 2) == 0) {
      state.preset_voltages[index] = f32(frame);
    } else {
      state.preset_currents[index] = f32(frame);
    }
    return true;
  }

  return false;
}

}  // namespace fnirsi_dps150_driver
