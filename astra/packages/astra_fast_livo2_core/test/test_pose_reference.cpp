#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "utils/pose_reference.h"

TEST(PoseReference, BodyCentreFollowsAttitudeInsteadOfWorldZ)
{
  const Eigen::Vector3d imu_position(1., 2., 3.);
  const Eigen::Vector3d offset(0., 0., -.12);
  const Eigen::Matrix3d pitch = Eigen::AngleAxisd(std::acos(-1.) / 2., Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto body = positionAtReference(imu_position, pitch, offset);
  EXPECT_NEAR(body.x(), .88, 1e-12);
  EXPECT_NEAR(body.y(), 2., 1e-12);
  EXPECT_NEAR(body.z(), 3., 1e-12);
  // Returning from the body reference recovers the unchanged IMU state.
  EXPECT_LT((positionAtReference(body, pitch, -offset)-imu_position).norm(), 1e-12);
}

TEST(PoseReference, ZeroOffsetPreservesLegacyPosition)
{
  const Eigen::Vector3d position(1., 2., 3.);
  EXPECT_EQ(positionAtReference(position, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()), position);
}
