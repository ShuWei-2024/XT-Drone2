#pragma once
#include <Eigen/Core>
#include <stdexcept>

namespace ego_planner {
inline void pinStationaryTerminal(Eigen::MatrixXd &controls, const Eigen::Vector3d &target)
{
  if (controls.rows() != 3 || controls.cols() < 7 || !target.allFinite())
    throw std::invalid_argument("stationary cubic terminal requires at least seven controls");
  // For a uniform cubic spline, three equal tail controls give exactly the
  // requested endpoint, zero terminal velocity and zero terminal acceleration.
  controls.rightCols(3) = target.replicate(1, 3);
}
}
