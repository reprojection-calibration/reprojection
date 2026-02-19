#include "spline/spline_initialization.hpp"

#include <gtest/gtest.h>

#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

using C3Measurement = spline::C3Measurement;
using CubicBSplineC3 = spline::CubicBSplineC3;
using DerivativeOrder = spline::DerivativeOrder;

// NOTE(Jack): In this test we do not get the exact start and end of the spline back like we might expect given the
// perfect input data. This happens because we do not handle the end conditions in a principled manner. Please see the
// implementation of CubicBSplineC3Init::BuildAb to understand this better. This is the reason why we get -2.0344...
// instead if just -2 for example in the tests below.
TEST(SplineSplineInitialization, TestInitializeSpline) {
    std::vector<C3Measurement> const measurements{{5000, {0, 0, 0}, DerivativeOrder::Null},  //
                                                  {5100, {1, 1, 1}, DerivativeOrder::Null},
                                                  {5200, {2, 2, 2}, DerivativeOrder::Null}};

    // ERROR(Jack): The InitializeC3Spline is visible even if we do not preface it with the spline:: namespace. Is there
    // something wrong I am doing?
    CubicBSplineC3 const one_segment_spline{spline::InitializeC3Spline(measurements, 1)};
    EXPECT_EQ(one_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(one_segment_spline.time_handler.delta_t_ns_, 200);
    EXPECT_EQ(std::size(one_segment_spline.control_points), 4);
    // NOTE(Jack): At this point this and below are canary in the coal mine tests, to make sure nothing changes as we
    // refactor. An unsolved problem is the time handling, and this is the reason why these values are not exact values
    // on the integers, which given the test data they should be.
    EXPECT_TRUE(one_segment_spline.control_points[0].isApprox(
        Vector3d{-2.0344390938914243, -2.0344390938914243, -2.0344390938914243}));
    EXPECT_TRUE(one_segment_spline.control_points[3].isApprox(
        Vector3d{4.0564567506989482, 4.0564567506989482, 4.0564567506989482}));

    CubicBSplineC3 const two_segment_spline{spline::InitializeC3Spline(measurements, 2)};
    EXPECT_EQ(two_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(two_segment_spline.time_handler.delta_t_ns_, 100);
    EXPECT_EQ(std::size(two_segment_spline.control_points), 5);
    // See note above on canary coal mine.
    EXPECT_TRUE(two_segment_spline.control_points[0].isApprox(
        Vector3d{-1.0132016841428102, -1.0132016841428102, -1.0132016841428102}));
    EXPECT_TRUE(two_segment_spline.control_points[4].isApprox(
        Vector3d{3.0270686114720653, 3.0270686114720653, 3.0270686114720653}, 1e-6));
}
