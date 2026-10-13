#include <gtest/gtest.h>
#include "snapshot_timing.h"

TEST(SnapshotTiming, NearbyFutureImageRequiresImuCoverage)
{
  EXPECT_TRUE(livo_time_covered(10.012, 10.0, 10.016, true, 0.03));
  EXPECT_FALSE(livo_time_covered(10.012, 10.0, 10.008, true, 0.03));
  EXPECT_FALSE(livo_time_covered(10.04, 10.0, 10.044, true, 0.03));
  EXPECT_FALSE(livo_time_covered(10.012, 10.0, 10.016, false, 0.03));
  EXPECT_TRUE(livo_time_covered(10.0, 10.0, 10.0, true, 0.03));
}
