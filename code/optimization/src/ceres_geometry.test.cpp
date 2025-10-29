#include <gtest/gtest.h>

#include "ceres_geometry.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(OptimizationCeresXxx, TestTransformPointsTranslation) {
    Eigen::Vector<double, 6> const tf{0, 0, 0, 1, 2, 3};  // Translation only
    Vector3d const point{5, 10, 15};

    Vector3d const transformed_point{optimization::TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], 6.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 12.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 18.0);
}

TEST(OptimizationCeresXxx, TestTransformPointsRotation) {
    Eigen::Vector<double, 6> const tf{0, 0, M_PI_2, 0, 0, 0};  // Rotation only
    Vector3d const point{5, 10, 15};

    Vector3d const transformed_point{optimization::TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], -10.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 5.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 15.0);
}
