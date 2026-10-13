#include "LIVMapper.h"
#include <csignal>
#include <thread>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("laserMapping", options);
  {
    image_transport::ImageTransport it_(nh);
    LIVMapper mapper(nh, "laserMapping");
    mapper.initializeSubscribersAndPublishers(nh, it_);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    std::thread receive_thread([&executor] { executor.spin(); });
    mapper.run(nh, stop_requested);
    executor.cancel();
    receive_thread.join();
    executor.remove_node(nh);
  }
  nh.reset();
  rclcpp::shutdown();
  return 0;
}
