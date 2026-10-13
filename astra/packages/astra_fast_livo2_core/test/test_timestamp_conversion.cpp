#include <gtest/gtest.h>
#include "utils/utils.h"

TEST(TimestampConversion, PreservesMeasuredStampAtInclusivePairBoundary)
{
  builtin_interfaces::msg::Time measured;
  measured.sec = 461;
  measured.nanosec = 904000000;
  const auto recovered = sec2Stamp(stamp2Sec(measured));
  EXPECT_EQ(recovered.nanoseconds(), 461904000000LL);
  const int64_t px4_sample = 461924000000LL;
  EXPECT_EQ(px4_sample - recovered.nanoseconds(), 20000000LL);
  EXPECT_LE(px4_sample - recovered.nanoseconds(), 20000000LL);
}

TEST(TimestampConversion, NormalizesFractionRoundedIntoNextSecond)
{
  const builtin_interfaces::msg::Time stamp = sec2Stamp(1.9999999996);
  EXPECT_EQ(stamp.sec, 2);
  EXPECT_EQ(stamp.nanosec, 0u);
}

TEST(TimestampConversion, PreservesMillisecondSensorTimeline)
{
  for (int64_t ns = 1000000; ns < 1000000000000LL; ns += 33000000)
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = ns / 1000000000LL;
    stamp.nanosec = ns % 1000000000LL;
    ASSERT_EQ(sec2Stamp(stamp2Sec(stamp)).nanoseconds(), ns);
  }
}
