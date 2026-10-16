#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <stdexcept>

namespace ego_planner {
// Knot scaling leaves the curve unchanged. Derivative control-point norms
// bound speed/acceleration throughout a B-spline, including between samples.
inline double retimeCollisionFreeSpline(UniformBspline &position,
                                      double velocity_limit, double acceleration_limit) {
  if (!std::isfinite(velocity_limit) || velocity_limit <= 0.0 ||
      !std::isfinite(acceleration_limit) || acceleration_limit <= 0.0 ||
      !position.getControlPoint().allFinite())
    throw std::invalid_argument("invalid spline motion limits or control points");
  auto velocity = position.getDerivative();
  const Eigen::MatrixXd v = velocity.getControlPoint();
  const Eigen::MatrixXd a = velocity.getDerivative().getControlPoint();
  if (!v.allFinite() || !a.allFinite())
    throw std::invalid_argument("nonfinite spline derivatives");
  const double scale = std::max({1.0, v.colwise().norm().maxCoeff() / velocity_limit,
      std::sqrt(a.colwise().norm().maxCoeff() / acceleration_limit)}) * 1.01;
  if (!std::isfinite(scale) || scale > 5.0)
    throw std::invalid_argument("excessive spline time scale");
  if (scale > 1.01) {
    Eigen::VectorXd knots = position.getKnot();
    const double origin = knots(3);
    knots = ((knots.array() - origin) * scale + origin).matrix();
    position.setKnot(knots);
  }
  return scale > 1.01 ? scale : 1.0;
}

// Uniform slowing alone reduces the initial velocity on every replan. Restore
// C0/C1/C2 at the head, bound the resulting derivative hull, and let the caller
// recheck occupancy of the changed head. The tail geometry remains unchanged.
// Keep the incoming turn acceleration when it is feasible. Callers may retry
// with a zero-acceleration braking prefix if a sharp turn would need excessive
// time scaling; that retry is still required to pass the complete occupancy
// and dynamic-limit checks.
inline double retimeStartContinuousSpline(UniformBspline &position,
    const Eigen::Vector3d &start, const Eigen::Vector3d &start_velocity,
    double velocity_limit, double acceleration_limit,
    const Eigen::Vector3d &start_acceleration=Eigen::Vector3d::Zero()) {
  if (!start.allFinite() || !start_velocity.allFinite() || !start_acceleration.allFinite() ||
      !std::isfinite(velocity_limit) || velocity_limit<=0. ||
      !std::isfinite(acceleration_limit) || acceleration_limit<=0. ||
      start_velocity.norm()>velocity_limit+1e-8 ||
      start_acceleration.norm()>acceleration_limit+1e-8)
    throw std::invalid_argument("invalid spline start state");
  const Eigen::MatrixXd controls=position.getControlPoint();
  const Eigen::VectorXd knots=position.getKnot();
  if (controls.cols()<6 || !controls.allFinite() || !knots.allFinite())
    throw std::invalid_argument("invalid spline controls or knots");
  const double dt=knots(4)-knots(3);
  if (!std::isfinite(dt) || dt<=0.) throw std::invalid_argument("invalid spline interval");
  double scale=1.;
  for (int attempt=0;attempt<16;++attempt) {
    if (!std::isfinite(scale) || scale>5.)
      throw std::invalid_argument("excessive continuous spline time scale");
    Eigen::MatrixXd updated=controls;
    const double interval=dt*scale;
    const Eigen::Vector3d acceleration_step=start_acceleration*interval*interval;
    updated.col(0)=start-start_velocity*interval+acceleration_step/3.;
    updated.col(1)=start-acceleration_step/6.;
    updated.col(2)=start+start_velocity*interval+acceleration_step/3.;
    position=UniformBspline(updated,3,dt*scale);
    position.setKnot(((knots.array()-knots(3))*scale+knots(3)).matrix());
    const double speed=position.getDerivative().getControlPoint().colwise().norm().maxCoeff();
    const double acceleration=position.getDerivative().getDerivative().getControlPoint().colwise().norm().maxCoeff();
    if (!std::isfinite(speed) || !std::isfinite(acceleration))
      throw std::invalid_argument("nonfinite continuous spline derivatives");
    if (speed<=velocity_limit+1e-9 && acceleration<=acceleration_limit+1e-9)
      return scale;
    scale*=std::max({1.,speed/velocity_limit,std::sqrt(acceleration/acceleration_limit)})*1.01;
  }
  throw std::invalid_argument("continuous spline retiming did not converge");
}
}
