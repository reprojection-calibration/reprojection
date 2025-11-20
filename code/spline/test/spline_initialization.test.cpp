#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

// We are building a linear problem Ax=b,
//      A: comes from our spline basis matrices
//      x: our unknown evenly spaced control points that created b - we want to find x
//      b: our measurements in the state space (same dimensions state space as the control points)
//
// The smallest problem we can build for a cubic b-spline is; given two measurements and two unique times, interpolate
// the four evenly spaced control points that define a spline which passes through the measurements.
//
// If we have two measurements then we can define one time segment with four control points, if we have three
// measurements we can either still define one time segment as before, or define two time segments now defined by five
// control points. How this works when measurements are missing, or too many or too few time segments are selected is at
// this time not clear.
//
// Our first strategy will be to stack all measurements and control points into vectors. Because we are dealing with a
// simple uniform spline it might happen that later we realize some symmetries that help us reduce the size of the
// problem itself. However, it might also be that because the measurements are not evenly spaced that this is not
// possible.
//      problem dimension: b_6_1 = A_6_12 * x_12_1      - for two measurements defining one time segment

namespace reprojection::spline {

// WARN(Jack): Expects time sorted measurements! Time stamp must be non-decreasing, how can we enforce this?
CubicBSplineC3 InitializeSpline(std::vector<C3Measurement> const& measurements, int const num_segments) {
    // TODO(Jack): Will rounding effect the time handling here?
    // TODO(Jack): Given a certain number of measurement is there a limit/boundary to valid num_segments?
    CubicBSplineC3 spline{measurements[0].t_ns, (measurements.back().t_ns - measurements.front().t_ns) / num_segments};

    return spline;
}

}  // namespace reprojection::spline

using namespace reprojection::spline;

TEST(SplineSplineInitialization, TestTimeHandling) {
    std::vector<C3Measurement> const measurements{{5000, {0, 0, 0}, DerivativeOrder::Null},  //
                                                  {5100, {1, 1, 1}, DerivativeOrder::Null},
                                                  {5200, {2, 2, 2}, DerivativeOrder::Null}};

    CubicBSplineC3 const one_segment_spline{InitializeSpline(measurements, 1)};
    EXPECT_EQ(one_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(one_segment_spline.time_handler.delta_t_ns_, 200);

    CubicBSplineC3 const two_segment_spline{InitializeSpline(measurements, 2)};
    EXPECT_EQ(two_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(two_segment_spline.time_handler.delta_t_ns_, 100);
}