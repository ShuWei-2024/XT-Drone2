#ifndef ASTRA_VIO_MEASUREMENT_ROWS_H
#define ASTRA_VIO_MEASUREMENT_ROWS_H

#include <Eigen/Core>

// Reset one point's fixed-size patch block before rebuilding its current
// image measurement. This prevents an out-of-frame patch from contributing
// stale residual/Jacobian values from an earlier EKF iteration.
inline void clearVisualMeasurementRows(Eigen::MatrixXd &jacobian,
                                      Eigen::VectorXd &residual,
                                      Eigen::Index first_row,
                                      Eigen::Index row_count,
                                      Eigen::Index state_columns)
{
  jacobian.block(first_row, 0, row_count, state_columns).setZero();
  residual.segment(first_row, row_count).setZero();
}

#endif
