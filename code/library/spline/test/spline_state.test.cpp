#include "spline/spline_state.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(SplineSplineState, TestSplineStateConstruction) {
    spline::TimeHandler const time_handler{100, 5};

    spline::MatrixNKd const control_points_matrix{
        {0, 1, 2, 3},  //
        {0, 1, 2, 3},
        {0, 1, 2, 3},
    };
    spline::CubicBSplineC3 const matrix_constructed{control_points_matrix, time_handler};
    EXPECT_TRUE(matrix_constructed.ControlPoints().isApprox(control_points_matrix));
}
