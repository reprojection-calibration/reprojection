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

    std::vector<Vector3d> const control_points_vector{{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    spline::CubicBSplineC3 const vector_constructed{control_points_vector, time_handler};
    EXPECT_TRUE(vector_constructed.ControlPoints().isApprox(control_points_matrix));
}

TEST(SplineSplineState, TestSplineStateHelperMethods) {
    spline::TimeHandler const time_handler{100, 5};
    spline::MatrixNXd const control_points_matrix{
        {0, 1, 2, 3},  //
        {0, 1, 2, 3},
        {0, 1, 2, 3},
    };
    spline::CubicBSplineC3 const spline{control_points_matrix, time_handler};

    EXPECT_EQ(spline.DeltaTNs(), time_handler.delta_t_ns_);
    EXPECT_EQ(spline.Size(), control_points_matrix.cols());

    // Only one segment in the spline.
    EXPECT_TRUE(spline.Position(100));
    EXPECT_TRUE(spline.Position(104));
    EXPECT_FALSE(spline.Position(105));
}
