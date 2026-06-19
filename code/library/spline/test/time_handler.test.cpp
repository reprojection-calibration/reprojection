#include "spline/time_handler.hpp"

#include <gtest/gtest.h>

#include "spline/constants.hpp"

using namespace reprojection;

TEST(SplineTimeHandler, TestTimeHandlerConstruction) {
    spline::TimeHandler const time_handler{100, 5};
    EXPECT_EQ(time_handler.t0_ns_, 100);
    EXPECT_EQ(time_handler.delta_t_ns_, 5);

    // delta_t_ns must be greater than zero!
    EXPECT_DEATH(spline::TimeHandler(0, 0), "");
}

TEST(SplineTimeHandler, TestTimeHandlerSplinePosition) {
    spline::TimeHandler const time_handler{100, 5};

    // t_ns is less than t0_ns.
    EXPECT_FALSE(time_handler.SplinePosition(0, 0));

    // Anything with less than K control points will fail.
    EXPECT_FALSE(time_handler.SplinePosition(100, 0));
    EXPECT_FALSE(time_handler.SplinePosition(100, spline::K - 1));

    // One valid time segment (K control points) from 100 to 104 with 105 being again invalid.
    EXPECT_TRUE(time_handler.SplinePosition(100, spline::K));
    EXPECT_TRUE(time_handler.SplinePosition(104, spline::K));
    EXPECT_FALSE(time_handler.SplinePosition(105, spline::K));

    // Two valid time segments (K+1 control points) from 100 to 109 with 110 being again invalid.
    EXPECT_TRUE(time_handler.SplinePosition(100, spline::K + 1));
    EXPECT_TRUE(time_handler.SplinePosition(109, spline::K + 1));
    EXPECT_FALSE(time_handler.SplinePosition(110, spline::K + 1));
}

TEST(SplineTimeHandler, TestNormalizedSegmentTime) {
    // Zero elapsed time edge case
    auto const [u1, i1]{spline::TimeHandler::NormalizedSegmentTime(100, 100, 5)};
    EXPECT_EQ(u1, 0);
    EXPECT_EQ(i1, 0);

    // From start of fourth segment (i=3) to 40% through it (0 -> 0.4)
    auto const [u2, i2]{spline::TimeHandler::NormalizedSegmentTime(100, 115, 5)};
    EXPECT_EQ(u2, 0);
    EXPECT_EQ(i2, 3);
    auto const [u3, i3]{spline::TimeHandler::NormalizedSegmentTime(100, 116, 5)};
    EXPECT_FLOAT_EQ(u3, 0.2);
    EXPECT_EQ(i3, 3);
    auto const [u4, i4]{spline::TimeHandler::NormalizedSegmentTime(100, 117, 5)};
    EXPECT_FLOAT_EQ(u4, 0.4);
    EXPECT_EQ(i4, 3);
}
