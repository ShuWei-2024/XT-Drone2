#include <gtest/gtest.h>
#include "LIVMapper.h"

TEST(PendingVio, CompletesWithoutNextCloudAndPreservesOtherInputGates)
{
  int argc = 0;
  char **argv = nullptr;
  rclcpp::init(argc, argv);
  {
    const std::vector<double> identity = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("extrin_calib.extrinsic_T", std::vector<double>{0., 0., 0.}),
        rclcpp::Parameter("extrin_calib.extrinsic_R", identity),
        rclcpp::Parameter("extrin_calib.Pcl", std::vector<double>{0., 0., 0.}),
        rclcpp::Parameter("extrin_calib.Rcl", identity),
        rclcpp::Parameter("preprocess.blind", 0.45),
        rclcpp::Parameter("preprocess.lidar_type", static_cast<int>(VELO16)),
        rclcpp::Parameter("preprocess.point_filter_num", 1),
        rclcpp::Parameter("preprocess.gazebo_snapshot", true)});
    auto node = std::make_shared<rclcpp::Node>("pending_vio_test", options);
    LIVMapper mapper(node, "pending_vio_test");
    // Exercise the ROS parameter path; setting blind_sqr in a test would hide
    // a missing update in readParameters().
    pcl::PointCloud<velodyne_ros::Point> cloud;
    for (const float distance : {0.2f, 0.3f, 0.6f, 0.8f})
    {
      velodyne_ros::Point point{};
      point.x = distance;
      cloud.push_back(point);
    }
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *cloud_msg);
    auto filtered = std::make_shared<PointCloudXYZI>();
    mapper.p_pre->process(cloud_msg, filtered);
    ASSERT_EQ(filtered->size(), 2u);
    EXPECT_FLOAT_EQ(filtered->front().x, 0.6f);
    EXPECT_FLOAT_EQ(filtered->back().x, 0.8f);
    mapper.slam_mode_ = LIVO;
    LidarMeasureGroup measure;
    measure.lio_vio_flg = LIO;
    measure.last_lio_update_time = 10.0;
    mapper.img_buffer.push_back(cv::Mat::zeros(1, 1, CV_8UC3));
    mapper.img_time_buffer.push_back(10.0);
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->header.stamp.sec = 10;
    imu->header.stamp.nanosec = 10000000;
    mapper.imu_buffer.push_back(imu);

    EXPECT_TRUE(mapper.lid_raw_data_buffer.empty());
    EXPECT_TRUE(mapper.sync_packages(measure));
    EXPECT_EQ(measure.lio_vio_flg, VIO);
    ASSERT_EQ(measure.measures.size(), 1u);
    EXPECT_DOUBLE_EQ(measure.measures.back().vio_time, 10.0);
    EXPECT_FALSE(measure.measures.back().img.empty());
    EXPECT_TRUE(mapper.img_buffer.empty());

    // The next LIO cut still requires a new cloud.
    mapper.img_buffer.push_back(cv::Mat::zeros(1, 1, CV_8UC3));
    mapper.img_time_buffer.push_back(10.1);
    EXPECT_FALSE(mapper.sync_packages(measure));
    measure.lio_vio_flg = LIO;
    mapper.slam_mode_ = ONLY_LIO;
    EXPECT_FALSE(mapper.sync_packages(measure));

    mapper.slam_mode_ = LIVO;
    mapper.imu_buffer.clear();
    EXPECT_FALSE(mapper.sync_packages(measure));
    mapper.imu_buffer.push_back(imu);
    mapper.img_buffer.clear();
    mapper.img_time_buffer.clear();
    EXPECT_FALSE(mapper.sync_packages(measure));
  }
  rclcpp::shutdown();
}
