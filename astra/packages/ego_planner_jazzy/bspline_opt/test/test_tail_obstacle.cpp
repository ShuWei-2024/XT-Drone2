// A finite obstacle near the trajectory tail must create rebound constraints.
#include <bspline_opt/bspline_optimizer.h>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void check_path(double obstacle_x, bool occupied, bool expect_tail) {
  auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("grid_map/resolution", 0.1),
      rclcpp::Parameter("grid_map/map_size_x", 12.0),
      rclcpp::Parameter("grid_map/map_size_y", 6.0),
      rclcpp::Parameter("grid_map/map_size_z", 3.0),
      rclcpp::Parameter("grid_map/local_update_range_x", 6.0),
      rclcpp::Parameter("grid_map/local_update_range_y", 3.0),
      rclcpp::Parameter("grid_map/local_update_range_z", 3.0),
      rclcpp::Parameter("grid_map/obstacles_inflation", 0.0),
      rclcpp::Parameter("grid_map/ground_height", 0.0),
      rclcpp::Parameter("grid_map/skip_pixel", 2),
      rclcpp::Parameter("optimization/dist0", 0.4)});
  auto node = std::make_shared<rclcpp::Node>("tail_obstacle_regression", options);
  auto map = std::make_shared<GridMap>();
  map->initMap(node);
  if (occupied) {
    for (double x = obstacle_x - .3; x <= obstacle_x + .3; x += .05)
      for (double y = -.3; y <= .3; y += .05)
        for (double z = .6; z <= 1.4; z += .05)
          map->setOccupied(Eigen::Vector3d(x, y, z));
  }
  ego_planner::BsplineOptimizer optimizer;
  optimizer.setParam(node);
  optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, Eigen::Vector3i(80, 80, 40));
  Eigen::MatrixXd path(3, 25);
  for (int i = 0; i < path.cols(); ++i)
    path.col(i) = Eigen::Vector3d(-4.8 + .4 * i, 0., 1.);
  const auto segments = optimizer.initControlPoints(path);
  require(!segments.empty() == occupied, "obstacle segmentation was incorrect");
  if (!occupied) return;
  const auto control = optimizer.getControlPoints();
  bool constrained = false;
  for (int i = 0; i < control.size; ++i) {
    if (expect_tail && i < 18) continue;
    for (const auto &direction : control.direction[i]) {
      require(direction.allFinite(), "rebound direction is non-finite");
      require(std::abs(direction.norm() - 1.) < 1e-6,
              "rebound direction is not normalized");
      constrained = true;
    }
  }
  require(constrained, "obstacle has no usable rebound constraints");
  if (expect_tail)
    require(segments.front().first >= 17,
            "test obstacle must lie beyond the old two-thirds cutoff");
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    check_path(2.8, false, false);
    check_path(-1.0, true, false);
    check_path(2.8, true, true);
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
