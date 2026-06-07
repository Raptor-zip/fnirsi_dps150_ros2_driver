#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fnirsi_dps150_driver
{
namespace protocol
{

constexpr std::uint8_t kHeaderRx = 0xF0;
constexpr std::uint8_t kHeaderTx = 0xF1;

constexpr std::uint8_t kCommandGet = 0xA1;
constexpr std::uint8_t kCommandBaud = 0xB0;
constexpr std::uint8_t kCommandSet = 0xB1;
constexpr std::uint8_t kCommandSession = 0xC1;

constexpr std::uint8_t kRegInputVoltage = 0xC0;
constexpr std::uint8_t kRegVoltageSet = 0xC1;
constexpr std::uint8_t kRegCurrentSet = 0xC2;
constexpr std::uint8_t kRegMeasurement = 0xC3;
constexpr std::uint8_t kRegTemperature = 0xC4;

constexpr std::uint8_t kRegPreset1Voltage = 0xC5;
constexpr std::uint8_t kRegPreset1Current = 0xC6;
constexpr std::uint8_t kRegPreset6Current = 0xD0;

constexpr std::uint8_t kRegOvp = 0xD1;
constexpr std::uint8_t kRegOcp = 0xD2;
constexpr std::uint8_t kRegOpp = 0xD3;
constexpr std::uint8_t kRegOtp = 0xD4;
constexpr std::uint8_t kRegLvp = 0xD5;
constexpr std::uint8_t kRegBrightness = 0xD6;
constexpr std::uint8_t kRegVolume = 0xD7;
constexpr std::uint8_t kRegMeteringEnable = 0xD8;
constexpr std::uint8_t kRegOutputCapacity = 0xD9;
constexpr std::uint8_t kRegOutputEnergy = 0xDA;
constexpr std::uint8_t kRegOutputEnable = 0xDB;
constexpr std::uint8_t kRegProtectionState = 0xDC;
constexpr std::uint8_t kRegMode = 0xDD;
constexpr std::uint8_t kRegModelName = 0xDE;
constexpr std::uint8_t kRegHardwareVersion = 0xDF;
constexpr std::uint8_t kRegFirmwareVersion = 0xE0;
constexpr std::uint8_t kRegUpperLimitVoltage = 0xE2;
constexpr std::uint8_t kRegUpperLimitCurrent = 0xE3;
constexpr std::uint8_t kRegAll = 0xFF;

struct Frame
{
  std::uint8_t header{};
  std::uint8_t command{};
  std::uint8_t reg{};
  std::vector<std::uint8_t> data;
  std::uint8_t checksum{};
};

struct ParseStats
{
  std::uint64_t checksum_errors{};
  std::uint64_t parse_errors{};
};

std::uint8_t checksum(std::uint8_t reg, const std::vector<std::uint8_t> & data);
std::vector<std::uint8_t> encode(
  std::uint8_t header,
  std::uint8_t command,
  std::uint8_t reg,
  const std::vector<std::uint8_t> & data);

std::vector<std::uint8_t> encode_byte(
  std::uint8_t header,
  std::uint8_t command,
  std::uint8_t reg,
  std::uint8_t value);

std::vector<std::uint8_t> encode_float(
  std::uint8_t header,
  std::uint8_t command,
  std::uint8_t reg,
  float value);

std::vector<std::uint8_t> float_to_little_endian(float value);
float little_endian_to_float(const std::vector<std::uint8_t> & data, std::size_t offset = 0);
std::string ascii_string(const std::vector<std::uint8_t> & data);

int baud_rate_code(int baud_rate);
std::uint8_t preset_voltage_register(std::uint8_t index);
std::uint8_t preset_current_register(std::uint8_t index);
bool is_preset_register(std::uint8_t reg);
bool is_protection_register(std::uint8_t reg);
const char * protection_name(std::uint8_t code);
const char * mode_name(std::uint8_t code);

class FrameParser
{
public:
  std::vector<Frame> feed(const std::uint8_t * data, std::size_t size);
  std::vector<Frame> feed(const std::vector<std::uint8_t> & data);

  const ParseStats & stats() const noexcept { return stats_; }
  void reset();

private:
  std::vector<std::uint8_t> buffer_;
  ParseStats stats_;
};

}  // namespace protocol
}  // namespace fnirsi_dps150_driver
