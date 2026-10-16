#pragma once
#include <bspline_opt/uniform_bspline.h>
#include <chrono>
#include <cmath>

namespace ego_planner {
// A failed replacement does not invalidate an unchanged, checked trajectory.
// Never extend its time span. Check the complete remaining suffix against the
// current map, with bounded work; uncertainty or budget exhaustion rejects it.
template <typename IsFree>
bool checkedReplanSuffix(UniformBspline &curve, double elapsed,
    double minimum_remaining, double sample_dt, IsFree is_free,
    std::chrono::steady_clock::time_point deadline) {
  const double end = curve.getTimeSum();
  if (!std::isfinite(elapsed) || !std::isfinite(end) || elapsed < 0. ||
      !std::isfinite(minimum_remaining) || minimum_remaining <= 0. ||
      end - elapsed <= minimum_remaining || !std::isfinite(sample_dt) ||
      sample_dt <= 0. || std::ceil((end - elapsed) / sample_dt) > 10000.) return false;
  for (double t = elapsed;; t = std::min(end, t + sample_dt)) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    const Eigen::Vector3d point = curve.evaluateDeBoorT(t);
    if (!point.allFinite() || !is_free(point)) return false;
    if (t >= end) return true;
  }
}
} // namespace ego_planner
