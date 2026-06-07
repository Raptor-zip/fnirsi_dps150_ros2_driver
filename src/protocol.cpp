#include "fnirsi_dps150_driver/protocol.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace fnirsi_dps150_driver
{
namespace protocol
{

std::uint8_t checksum(std::uint8_t reg, const std::vector<std::uint8_t> & data)
{
  std::uint32_t sum = reg + static_cast<std::uint8_t>(data.size());
  for (const auto byte : data) {
    sum += byte;
  }
  return static_cast<std::uint8_t>(sum & 0xFF);
}

std::vector<std::uint8_t> encode(
  std::uint8_t header,
  std::uint8_t command,
  std::uint8_t reg,
  const std::vector<std::uint8_t> & data)
{
  if (data.size() > 255) {
    throw std::invalid_argument("DPS-150 frames cannot carry more than 255 data bytes");
  }

  std::vector<std::uint8_t> frame;
  frame.reserve(data.size() + 5);
  frame.push_back(header);
  frame.push_back(command);
  frame.push_back(reg);
  frame.push_back(static_cast<std::uint8_t>(data.size()));
  frame.insert(frame.end(), data.begin(), data.end());
  frame.push_back(checksum(reg, data));
  return frame;
}

std::vector<std::uint8_t> encode_byte(
  std::uint8_t header,
  std::uint8_t command,
  std::uint8_t reg,
  std::uint8_t value)
{
  return encode(header, command, reg, std::vector<std::uint8_t>{value});
}

std::vector<std::uint8_t> encode_float(
  std::uint8_t header,
  std::uint8_t command,
  std::uint8_t reg,
  float value)
{
  return encode(header, command, reg, float_to_little_endian(value));
}

std::vector<std::uint8_t> float_to_little_endian(float value)
{
  std::uint32_t raw = 0;
  static_assert(sizeof(raw) == sizeof(value), "float32 is required");
  std::memcpy(&raw, &value, sizeof(raw));

  return {
    static_cast<std::uint8_t>(raw & 0xFF),
    static_cast<std::uint8_t>((raw >> 8) & 0xFF),
    static_cast<std::uint8_t>((raw >> 16) & 0xFF),
    static_cast<std::uint8_t>((raw >> 24) & 0xFF),
  };
}

float little_endian_to_float(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  if (offset + 4 > data.size()) {
    throw std::out_of_range("not enough bytes for float32");
  }

  const std::uint32_t raw =
    static_cast<std::uint32_t>(data[offset]) |
    (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
    (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
    (static_cast<std::uint32_t>(data[offset + 3]) << 24);

  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

std::string ascii_string(const std::vector<std::uint8_t> & data)
{
  auto end = std::find(data.begin(), data.end(), 0);
  return std::string(data.begin(), end);
}

int baud_rate_code(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return 1;
    case 19200:
      return 2;
    case 38400:
      return 3;
    case 57600:
      return 4;
    case 115200:
      return 5;
    default:
      throw std::invalid_argument("unsupported DPS-150 session baud rate");
  }
}

std::uint8_t preset_voltage_register(std::uint8_t index)
{
  if (index < 1 || index > 6) {
    throw std::out_of_range("preset index must be in the range 1..6");
  }
  return static_cast<std::uint8_t>(kRegPreset1Voltage + ((index - 1) * 2));
}

std::uint8_t preset_current_register(std::uint8_t index)
{
  if (index < 1 || index > 6) {
    throw std::out_of_range("preset index must be in the range 1..6");
  }
  return static_cast<std::uint8_t>(kRegPreset1Current + ((index - 1) * 2));
}

bool is_preset_register(std::uint8_t reg)
{
  return reg >= kRegPreset1Voltage && reg <= kRegPreset6Current;
}

bool is_protection_register(std::uint8_t reg)
{
  return reg >= kRegOvp && reg <= kRegLvp;
}

const char * protection_name(std::uint8_t code)
{
  switch (code) {
    case 0:
      return "OK";
    case 1:
      return "OVP";
    case 2:
      return "OCP";
    case 3:
      return "OPP";
    case 4:
      return "OTP";
    case 5:
      return "LVP";
    case 6:
      return "REP";
    default:
      return "UNKNOWN";
  }
}

const char * mode_name(std::uint8_t code)
{
  switch (code) {
    case 0:
      return "CC";
    case 1:
      return "CV";
    default:
      return "UNKNOWN";
  }
}

std::vector<Frame> FrameParser::feed(const std::uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);

  std::vector<Frame> frames;
  while (true) {
    const auto header_it = std::find(buffer_.begin(), buffer_.end(), kHeaderRx);
    if (header_it == buffer_.end()) {
      if (!buffer_.empty()) {
        ++stats_.parse_errors;
      }
      buffer_.clear();
      break;
    }

    if (header_it != buffer_.begin()) {
      ++stats_.parse_errors;
      buffer_.erase(buffer_.begin(), header_it);
    }

    if (buffer_.size() < 5) {
      break;
    }

    const auto command = buffer_[1];
    if (command != kCommandGet) {
      ++stats_.parse_errors;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t length = buffer_[3];
    const std::size_t total_size = length + 5;
    if (buffer_.size() < total_size) {
      break;
    }

    Frame frame;
    frame.header = buffer_[0];
    frame.command = command;
    frame.reg = buffer_[2];
    frame.data.assign(buffer_.begin() + 4, buffer_.begin() + 4 + length);
    frame.checksum = buffer_[total_size - 1];

    if (checksum(frame.reg, frame.data) != frame.checksum) {
      ++stats_.checksum_errors;
      buffer_.erase(buffer_.begin());
      continue;
    }

    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
  }

  return frames;
}

std::vector<Frame> FrameParser::feed(const std::vector<std::uint8_t> & data)
{
  return feed(data.data(), data.size());
}

void FrameParser::reset()
{
  buffer_.clear();
  stats_ = {};
}

}  // namespace protocol
}  // namespace fnirsi_dps150_driver
