#pragma once
#include <Eigen/Core>

// Offset is expressed in the estimator's IMU axes, not world axes.
inline Eigen::Vector3d positionAtReference(const Eigen::Vector3d &imu_position,
                                         const Eigen::Matrix3d &world_from_imu,
                                         const Eigen::Vector3d &imu_to_reference)
{
  return imu_position + world_from_imu * imu_to_reference;
}
