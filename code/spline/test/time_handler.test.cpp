#include "spline/time_handler.hpp"

#include <gtest/gtest.h>

using namespace reprojection::spline;

TEST(SplineUtilities, TestNormalizedSegmentTime) {
    // Zero elapsed time edge case
    auto const [u1, i1]{NormalizedSegmentTime(100, 100, 5)};
    EXPECT_EQ(u1, 0);
    EXPECT_EQ(i1, 0);

    // From start of fourth segment (i=3) to 40% through it (0 -> 0.4)
    auto const [u2, i2]{NormalizedSegmentTime(100, 115, 5)};
    EXPECT_EQ(u2, 0);
    EXPECT_EQ(i2, 3);
    auto const [u3, i3]{NormalizedSegmentTime(100, 116, 5)};
    EXPECT_FLOAT_EQ(u3, 0.2);
    EXPECT_EQ(i3, 3);
    auto const [u4, i4]{NormalizedSegmentTime(100, 117, 5)};
    EXPECT_FLOAT_EQ(u4, 0.4);
    EXPECT_EQ(i4, 3);
}
