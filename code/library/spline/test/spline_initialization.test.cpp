#include "spline/spline_initialization.hpp"

#include <gtest/gtest.h>

#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(SplineSplineInitialization, TestBuildOmega) {
    MatrixXd const Q_1{spline::BuildOmega(1, 1)};
    EXPECT_EQ(Q_1.rows(), 12);
    EXPECT_EQ(Q_1.cols(), 12);
    EXPECT_FLOAT_EQ(Q_1.diagonal().sum(), 8);  // Heuristic!

    MatrixXd const Q_100{spline::BuildOmega(100, 1)};
    EXPECT_FLOAT_EQ(Q_100.diagonal().sum(), 8e-6);
}