#include "spline/spline_initialization.hpp"

#include <gtest/gtest.h>

#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

using CubicBSplineC3 = spline::CubicBSplineC3;
using DerivativeOrder = spline::DerivativeOrder;

// NOTE(Jack): In this test we do not get the exact start and end of the spline back like we might expect given the
// perfect input data. This happens because we do not handle the end conditions in a principled manner. Please see the
// implementation of CubicBSplineC3Init::BuildAb to understand this better. This is the reason why we get -2.0344...
// instead if just -2 for example in the tests below.
TEST(SplineSplineInitialization, TestInitializeSpline) {
    PositionMeasurements const measurements{{5000, {{0, 0, 0}}},  //
                                            {5100, {{1, 1, 1}}},
                                            {5200, {{2, 2, 2}}}};

    // ERROR(Jack): The InitializeC3Spline is visible even if we do not preface it with the spline:: namespace. Is there
    // something wrong I am doing?
    CubicBSplineC3 const one_segment_spline{spline::InitializeC3Spline(measurements, 1)};
    EXPECT_EQ(one_segment_spline.time_handler_.t0_ns_, 5000);
    EXPECT_EQ(one_segment_spline.time_handler_.delta_t_ns_, 200);
    EXPECT_EQ(one_segment_spline.Size(), 4);
    // NOTE(Jack): At this point this and below are canary in the coal mine tests, to make sure nothing changes as we
    // refactor. An unsolved problem is the time handling, and this is the reason why these values are not exact values
    // on the integers, which given the test data they should be.
    EXPECT_TRUE(one_segment_spline.ControlPoints().col(0).isApprox(
        Vector3d{-2.0352118285642522, -2.0352118285642522, -2.0352118285642522}));
    EXPECT_TRUE(one_segment_spline.ControlPoints().col(3).isApprox(
        Vector3d{4.0556875027223356, 4.0556875027223356, 4.0556875027223356}));

    CubicBSplineC3 const two_segment_spline{spline::InitializeC3Spline(measurements, 2)};
    EXPECT_EQ(two_segment_spline.time_handler_.t0_ns_, 5000);
    EXPECT_EQ(two_segment_spline.time_handler_.delta_t_ns_, 100);
    EXPECT_EQ(two_segment_spline.Size(), 5);
    // See note above on canary coal mine.
    EXPECT_TRUE(two_segment_spline.ControlPoints().col(0).isApprox(
        Vector3d{-1.0133795604206075, -1.0133795604206075, -1.0133795604206075}));
    EXPECT_TRUE(two_segment_spline.ControlPoints().col(4).isApprox(
        Vector3d{3.0268874400439727, 3.0268874400439727, 3.0268874400439727}, 1e-6));
}
