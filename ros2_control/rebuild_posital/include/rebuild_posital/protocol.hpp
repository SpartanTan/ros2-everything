#pragma once

#include <cstdint>

namespace rebuild_posital
{

constexpr std::uint16_t kSlopeLong16Index = 0x6010;
constexpr std::uint16_t kTpdo2CommunicationIndex = 0x1801;

double decode_pitch_rad(
  std::uint16_t raw_value, double counts_per_degree, double direction,
  double mounting_offset_rad);

}  // namespace rebuild_posital
