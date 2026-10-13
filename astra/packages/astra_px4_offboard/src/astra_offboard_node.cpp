#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using namespace std::chrono_literals;

class AstraOffboardNode final : public rclcpp::Node
{
public:
  AstraOffboardNode()
  : Node("astra_offboard_node"), cycle_(0), phase_(Phase::PRESTREAM)
  {
    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    timer_ = create_wall_timer(100ms, std::bind(&AstraOffboardNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "Astra PX4 Offboard test started: pre-stream, arm, hover, move, land");
  }

private:
  enum class Phase { PRESTREAM, HOVER, MOVE_X, MOVE_Y, LAND, DONE };

  static constexpr float nan_value() { return std::numeric_limits<float>::quiet_NaN(); }

  uint64_t timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void publish_offboard_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = timestamp_us();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_pub_->publish(msg);
  }

  void publish_setpoint(float north, float east, float down)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = timestamp_us();
    msg.position = {north, east, down};
    msg.velocity = {nan_value(), nan_value(), nan_value()};
    msg.acceleration = {nan_value(), nan_value(), nan_value()};
    msg.jerk = {nan_value(), nan_value(), nan_value()};
    msg.yaw = 0.0F;
    msg.yawspeed = nan_value();
    setpoint_pub_->publish(msg);
  }

  void publish_command(uint32_t command, float p1 = 0.0F, float p2 = 0.0F, float p3 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.param1 = p1;
    msg.param2 = p2;
    msg.param3 = p3;
    msg.param4 = 0.0F;
    msg.param5 = 0.0;
    msg.param6 = 0.0;
    msg.param7 = 0.0F;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.confirmation = 0;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  void on_timer()
  {
    publish_offboard_mode();

    // PX4 expects a continuous position setpoint stream before entering Offboard.
    switch (phase_) {
      case Phase::PRESTREAM:
        publish_setpoint(0.0F, 0.0F, -2.0F);
        if (++cycle_ == 20) {
          publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
          publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
          phase_ = Phase::HOVER;
          cycle_ = 0;
          RCLCPP_INFO(get_logger(), "Requested PX4 Offboard mode and arm");
        }
        break;
      case Phase::HOVER:
        publish_setpoint(0.0F, 0.0F, -2.0F);
        if (++cycle_ == 50) {
          phase_ = Phase::MOVE_X;
          cycle_ = 0;
          RCLCPP_INFO(get_logger(), "Moving to NED setpoint (2, 0, -2)");
        }
        break;
      case Phase::MOVE_X:
        publish_setpoint(2.0F, 0.0F, -2.0F);
        if (++cycle_ == 50) {
          phase_ = Phase::MOVE_Y;
          cycle_ = 0;
          RCLCPP_INFO(get_logger(), "Moving to NED setpoint (2, 2, -2)");
        }
        break;
      case Phase::MOVE_Y:
        publish_setpoint(2.0F, 2.0F, -2.0F);
        if (++cycle_ == 50) {
          publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
          phase_ = Phase::LAND;
          cycle_ = 0;
          RCLCPP_INFO(get_logger(), "Requested PX4 landing");
        }
        break;
      case Phase::LAND:
        // Keep publishing until PX4 accepts the land command and exits Offboard.
        publish_setpoint(2.0F, 2.0F, -2.0F);
        if (++cycle_ == 100) {
          publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
          phase_ = Phase::DONE;
          RCLCPP_INFO(get_logger(), "Requested disarm after landing interval");
        }
        break;
      case Phase::DONE:
        publish_setpoint(2.0F, 2.0F, -2.0F);
        break;
    }
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t cycle_;
  Phase phase_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstraOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
