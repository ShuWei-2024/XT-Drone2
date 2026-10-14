#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <stdexcept>

namespace ego_planner {
// Convert a sensor-grid route to a cubic curve with specified endpoint states.
// The caller must certify occupancy after construction and retiming.
inline UniformBspline gridPathSpline(const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration,
    const Eigen::Vector3d &goal_velocity, double spacing, double dt) {
  if (path.size()<2 || path.size()>2048 || !std::isfinite(spacing) || spacing<=0. ||
      !std::isfinite(dt) || dt<=0. || !velocity.allFinite() || !acceleration.allFinite() ||
      !goal_velocity.allFinite()) throw std::invalid_argument("invalid grid spline inputs");
  for (const auto &p:path)
    if (!p.allFinite()) throw std::invalid_argument("nonfinite grid path");
  std::vector<Eigen::Vector3d> points;
  const Eigen::Vector3d middle=path.front()-acceleration*dt*dt/6.;
  points.push_back(middle-velocity*dt+acceleration*dt*dt/2.);
  points.push_back(middle);
  points.push_back(middle+velocity*dt+acceleration*dt*dt/2.);
  // The first free control follows the boundary control at start+velocity*dt.
  // Starting samples at spacing alone inserts a short braking span on each
  // replan. Place the first sample another spacing beyond that boundary.
  double remaining=spacing+velocity.norm()*dt;
  for (size_t i=1; i<path.size(); ++i) {
    Eigen::Vector3d from=path[i-1];
    double length=(path[i]-from).norm();
    while (length+1e-10>=remaining && length>1e-10) {
      from+=(path[i]-from)*(std::min(remaining,length)/length);
      points.push_back(from);
      length=(path[i]-from).norm();
      remaining=spacing;
    }
    remaining-=length;
    if (remaining<1e-10) remaining=spacing;
  }
  const Eigen::Vector3d goal=path.back();
  points.push_back(goal-goal_velocity*dt);
  points.push_back(goal);
  points.push_back(goal+goal_velocity*dt);
  Eigen::MatrixXd controls(3,points.size());
  for (size_t i=0;i<points.size();++i) controls.col(i)=points[i];
  return UniformBspline(controls,3,dt);
}
}
