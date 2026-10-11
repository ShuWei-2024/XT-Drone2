#pragma once
#include <cmath>
#include <Eigen/Eigenvalues>

inline bool ordered_plane_spectrum(const Eigen::Matrix3d &covariance,
                                   Eigen::Vector3d &values, Eigen::Matrix3d &vectors)
{
  if (!covariance.allFinite()) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) return false;
  values = solver.eigenvalues();
  vectors = solver.eigenvectors();
  // A repeated smallest eigenvalue has no unique normal and makes the
  // normal covariance denominator zero. Reject it before division.
  return values.allFinite() && vectors.allFinite() &&
         values[1] - values[0] > 1e-12;
}

inline bool plane_spectrum_accepted(double smallest, double middle, double largest,
                                    double absolute_limit, double max_plane_ratio,
                                    double min_extent_ratio)
{
  if (!(smallest < absolute_limit)) return false;
  if (max_plane_ratio <= 0. && min_extent_ratio <= 0.) return true;
  if (!std::isfinite(smallest) || !std::isfinite(middle) || !std::isfinite(largest) ||
      largest <= 1e-12 || middle <= 1e-12 || smallest < -1e-12) return false;
  return (max_plane_ratio <= 0. || smallest <= max_plane_ratio * middle) &&
         (min_extent_ratio <= 0. || middle >= min_extent_ratio * largest);
}
