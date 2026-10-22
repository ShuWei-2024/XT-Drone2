#include <gtest/gtest.h>
#include <Eigen/Core>
#include "vio_measurement_rows.h"

TEST(VioMeasurementRows, ClearsOnlyTheCurrentPatchBlock)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Constant(12, 7, 3.0);
  Eigen::VectorXd residual = Eigen::VectorXd::Constant(12, -2.0);

  clearVisualMeasurementRows(jacobian, residual, 4, 4, 7);

  EXPECT_TRUE(jacobian.block(0, 0, 4, 7).isConstant(3.0));
  EXPECT_TRUE(jacobian.block(4, 0, 4, 7).isZero());
  EXPECT_TRUE(jacobian.block(8, 0, 4, 7).isConstant(3.0));
  EXPECT_TRUE(residual.segment(0, 4).isConstant(-2.0));
  EXPECT_TRUE(residual.segment(4, 4).isZero());
  EXPECT_TRUE(residual.segment(8, 4).isConstant(-2.0));
}

TEST(VioMeasurementRows, ClearsTheSixDimensionalStateWithoutTouchingOtherStorage)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Constant(8, 7, 3.0);
  Eigen::VectorXd residual = Eigen::VectorXd::Constant(8, 2.0);

  clearVisualMeasurementRows(jacobian, residual, 0, 4, 6);

  EXPECT_TRUE(jacobian.block(0, 0, 4, 6).isZero());
  EXPECT_TRUE(jacobian.block(0, 6, 4, 1).isConstant(3.0));
  EXPECT_TRUE(jacobian.block(4, 0, 4, 7).isConstant(3.0));
  EXPECT_TRUE(residual.segment(0, 4).isZero());
  EXPECT_TRUE(residual.segment(4, 4).isConstant(2.0));
}
