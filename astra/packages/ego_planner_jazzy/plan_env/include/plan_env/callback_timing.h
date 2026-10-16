#pragma once
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace ego_planner {
// Uses wall time even when /clock stops. A slow callback must not be hidden by
// publishing a heartbeat from another thread.
class CallbackTiming {
 public:
  CallbackTiming(rclcpp::Logger logger, const char *name)
      : logger_(logger), name_(name), start_(std::chrono::steady_clock::now()) {}
  ~CallbackTiming() {
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_).count();
    if (ms > 40.0)
      RCLCPP_WARN(logger_, "Planner callback timing: %s wall_ms=%.3f", name_, ms);
  }
 private:
  rclcpp::Logger logger_;
  const char *name_;
  std::chrono::steady_clock::time_point start_;
};
}
