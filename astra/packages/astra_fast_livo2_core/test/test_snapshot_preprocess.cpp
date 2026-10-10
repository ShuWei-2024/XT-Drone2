#include <gtest/gtest.h>
#include "preprocess.h"

TEST(SnapshotPreprocess, ZeroOffsetsAreNotReconstructedAsRotation)
{
  pcl::PointCloud<velodyne_ros::Point> input;
  for (int i = 0; i < 8; ++i)
  {
    velodyne_ros::Point p{};
    const double angle = -0.2 * i;
    p.x = 3.0 * std::cos(angle);
    p.y = 3.0 * std::sin(angle);
    p.z = 0.0;
    p.ring = 0;
    p.time = 0.0;
    input.push_back(p);
  }
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(input, *msg);
  auto output = std::make_shared<PointCloudXYZI>();
  Preprocess pre;
  pre.set(false, VELO16, 0.1, 1);
  pre.gazebo_snapshot = true;
  pre.process(msg, output);
  ASSERT_EQ(output->size(), input.size());
  for (const auto &point : *output) EXPECT_FLOAT_EQ(point.curvature, 0.0f);
  pre.gazebo_snapshot = false;
  pre.process(msg, output);
  ASSERT_FALSE(output->empty());
  EXPECT_GT(output->back().curvature, 0.0f);
}

TEST(SnapshotPreprocess, RangeFilterTracksConfiguredBlindDistance)
{
  pcl::PointCloud<velodyne_ros::Point> input;
  for (const float distance : {0.005f, 0.02f, 0.2f, 0.3f, 0.6f, 0.8f})
  {
    velodyne_ros::Point point{};
    point.x = distance;
    input.push_back(point);
  }
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(input, *msg);
  auto output = std::make_shared<PointCloudXYZI>();
  Preprocess pre;
  pre.lidar_type = VELO16;
  pre.gazebo_snapshot = true;
  pre.process(msg, output);
  ASSERT_EQ(output->size(), 5u);  // Constructor's default 0.01 m filter.
  EXPECT_FLOAT_EQ(output->front().x, 0.02f);

  pre.set(false, VELO16, 0.45, 1);
  pre.process(msg, output);
  ASSERT_EQ(output->size(), 2u);
  EXPECT_FLOAT_EQ(output->front().x, 0.6f);
  EXPECT_FLOAT_EQ(output->back().x, 0.8f);

  pre.set(false, VELO16, 0.1, 1);
  pre.process(msg, output);
  ASSERT_EQ(output->size(), 4u);
  EXPECT_FLOAT_EQ(output->front().x, 0.2f);
}
