#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "fnirsi_dps150_driver/device_state.hpp"
#include "fnirsi_dps150_driver/protocol.hpp"

namespace
{

using fnirsi_dps150_driver::DeviceState;
namespace protocol = fnirsi_dps150_driver::protocol;

TEST(Protocol, EncodesSessionEnable)
{
  const auto frame = protocol::encode_byte(
    protocol::kHeaderTx,
    protocol::kCommandSession,
    0x00,
    0x01);

  const std::vector<std::uint8_t> expected{0xF1, 0xC1, 0x00, 0x01, 0x01, 0x02};
  EXPECT_EQ(frame, expected);
}

TEST(Protocol, EncodesFloatCommand)
{
  const auto frame = protocol::encode_float(
    protocol::kHeaderTx,
    protocol::kCommandSet,
    protocol::kRegVoltageSet,
    12.3F);

  const std::vector<std::uint8_t> expected{0xF1, 0xB1, 0xC1, 0x04, 0xCD, 0xCC, 0x44, 0x41, 0xE3};
  EXPECT_EQ(frame, expected);
}

TEST(Protocol, ParsesSplitFramesAndSkipsBadChecksum)
{
  protocol::FrameParser parser;
  const auto voltage = protocol::encode_float(
    protocol::kHeaderRx,
    protocol::kCommandGet,
    protocol::kRegInputVoltage,
    12.5F);
  auto bad = protocol::encode_float(
    protocol::kHeaderRx,
    protocol::kCommandGet,
    protocol::kRegTemperature,
    30.0F);
  bad.back() = 0xFF;

  std::vector<std::uint8_t> first_chunk;
  first_chunk.insert(first_chunk.end(), bad.begin(), bad.end());
  first_chunk.insert(first_chunk.end(), voltage.begin(), voltage.begin() + 3);
  EXPECT_TRUE(parser.feed(first_chunk).empty());

  std::vector<std::uint8_t> second_chunk(voltage.begin() + 3, voltage.end());
  const auto frames = parser.feed(second_chunk);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].reg, protocol::kRegInputVoltage);
  EXPECT_FLOAT_EQ(protocol::little_endian_to_float(frames[0].data), 12.5F);
  EXPECT_EQ(parser.stats().checksum_errors, 1U);
}

TEST(DeviceState, AppliesFullDump)
{
  DeviceState state;
  std::vector<std::uint8_t> data(139, 0);

  auto put_float = [&data](std::size_t offset, float value) {
      const auto encoded = protocol::float_to_little_endian(value);
      data[offset] = encoded[0];
      data[offset + 1] = encoded[1];
      data[offset + 2] = encoded[2];
      data[offset + 3] = encoded[3];
    };

  put_float(0, 20.0F);
  put_float(4, 5.0F);
  put_float(8, 1.5F);
  put_float(12, 4.95F);
  put_float(16, 1.25F);
  put_float(20, 6.1875F);
  put_float(24, 31.0F);
  put_float(28, 3.3F);
  put_float(32, 0.5F);
  put_float(76, 21.0F);
  put_float(80, 5.1F);
  put_float(84, 150.0F);
  put_float(88, 75.0F);
  put_float(92, 4.0F);
  data[96] = 12;
  data[97] = 9;
  data[98] = 1;
  put_float(99, 0.25F);
  put_float(103, 1.5F);
  data[107] = 0;
  data[108] = 2;
  data[109] = 1;
  put_float(111, 19.8F);
  put_float(115, 5.1F);

  const protocol::Frame frame{
    protocol::kHeaderRx,
    protocol::kCommandGet,
    protocol::kRegAll,
    data,
    protocol::checksum(protocol::kRegAll, data)};

  ASSERT_TRUE(fnirsi_dps150_driver::apply_frame_to_state(frame, state));
  EXPECT_FLOAT_EQ(state.input_voltage, 20.0F);
  EXPECT_FLOAT_EQ(state.set_voltage, 5.0F);
  EXPECT_FLOAT_EQ(state.preset_voltages[0], 3.3F);
  EXPECT_FLOAT_EQ(state.preset_currents[0], 0.5F);
  EXPECT_EQ(state.brightness, 12);
  EXPECT_EQ(state.volume, 9);
  EXPECT_TRUE(state.metering_enabled);
  EXPECT_FALSE(state.output_enabled);
  EXPECT_EQ(state.protection, "OCP");
  EXPECT_EQ(state.mode, "CV");
  EXPECT_FLOAT_EQ(state.upper_limit_voltage, 19.8F);
  EXPECT_FLOAT_EQ(state.upper_limit_current, 5.1F);
}

}  // namespace
