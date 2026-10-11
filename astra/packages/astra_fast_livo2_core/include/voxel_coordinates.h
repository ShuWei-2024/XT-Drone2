#pragma once
#include <cmath>
#include <cstdint>

// Use metres for geometric comparisons, and floor for integer voxel keys.
inline std::int64_t voxel_coordinate(double position_m, double voxel_size_m)
{
  return static_cast<std::int64_t>(std::floor(position_m / voxel_size_m));
}

inline int neighbouring_voxel_offset(double position_m, double centre_m, double quarter_length_m)
{
  if (position_m > centre_m + quarter_length_m) return 1;
  if (position_m < centre_m - quarter_length_m) return -1;
  return 0;
}
