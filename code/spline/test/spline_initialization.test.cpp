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

CubicBSplineC3 InitializeSpline(std::vector<C3Measurement> const& measurements);

}

using namespace reprojection;

TEST(SplineSplineInitialization, TestXxx) { EXPECT_EQ(1, 2); }