#include "ceres_geometry.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(OptimizationCeresGeometry, TestTransformPointTranslation) {
    Vector6d const tf_i_j{0, 0, 0, 1, 2, 3};  // Translation only
    Vector3d const point_j{5, 10, 15};

    Vector3d const point_i{optimization::TransformPoint<double>(tf_i_j, point_j)};

    EXPECT_TRUE(point_i.isApprox(Vector3d{6, 12, 18}));
}

TEST(OptimizationCeresGeometry, TestTransformPointRotation) {
    Vector6d const tf_i_j{0, 0, M_PI_2, 0, 0, 0};  // Rotation only
    Vector3d const point_j{5, 10, 15};

    Vector3d const point_i{optimization::TransformPoint<double>(tf_i_j, point_j)};

    EXPECT_TRUE(point_i.isApprox(Vector3d{-10, 5, 15}));
}

TEST(OptimizationCeresGeometry, TestTransformPointHeuristic) {
    // Random data with a heuristic expected result so we can hopefully catch regressions.
    Vector6d const tf_i_j{0.1, 0.2, 0.3, 1, 2, 3};
    Vector3d const point_j{4, 5, 6};

    Vector3d const point_i{optimization::TransformPoint<double>(tf_i_j, point_j)};

    EXPECT_TRUE(point_i.isApprox(Vector3d{4.5883446459907642, 7.5564460447113655, 8.7662544215288349}));
}