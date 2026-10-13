#pragma once

// Snapshot points can be propagated to a nearby camera time with IMU coverage.
inline bool livo_time_covered(double image_time, double lidar_time, double imu_time,
                              bool snapshot, double max_image_delta)
{
  return image_time <= imu_time &&
         (image_time <= lidar_time ||
          (snapshot && image_time - lidar_time <= max_image_delta));
}
