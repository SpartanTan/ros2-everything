#include "rebuild_posital/protocol.hpp"

#include <cmath>
#include <stdexcept>

namespace rebuild_posital
{
constexpr double kPi = 3.14159265358979323846;

double normalize_radians(double angle)
{
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0)
  {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

double decode_pitch_rad(std::uint16_t raw_value, double counts_per_degree, double direction,
                        double mounting_offset_rad)
{
  if (!std::isfinite(counts_per_degree) || counts_per_degree <= 0.0)
  {
    throw std::invalid_argument("counts_per_degree must be finite and greater than zero");
  }
  if (!std::isfinite(direction) || direction == 0.0)
  {
    throw std::invalid_argument("direction must be finite and non-zero");
  }

  const double raw_angle_rad = (static_cast<double>(raw_value) / counts_per_degree) * kPi / 180.0;
  return normalize_radians(direction * raw_angle_rad + mounting_offset_rad);
}
}  // namespace rebuild_posital