#include <gtest/gtest.h>

#include "reprojection/geometric_transforms.hpp"

using namespace reprojection_calibration::reprojection;

TEST(TestGeometricTransforms, NoTransform) {
    std::array<double, 6> const tf{0, 0, 0, 0, 0, 0};
    std::array<double, 3> const point{5, 10, 15};

    std::array<double, 3> const transformed_point{TransformPoint(tf.data(), point.data())};

    EXPECT_FLOAT_EQ(transformed_point[0], 5.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 10.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 15.0);
}

TEST(TestGeometricTransforms, TranslationOnly) {
    std::array<double, 6> const tf{0, 0, 0, 1, 2, 3};
    std::array<double, 3> const point{5, 10, 15};

    std::array<double, 3> const transformed_point{TransformPoint(tf.data(), point.data())};

    EXPECT_FLOAT_EQ(transformed_point[0], 6.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 12.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 18.0);
}

TEST(TestGeometricTransforms, RotationOnlz) {
    std::array<double, 6> const tf{0, 0, M_PI_2, 0, 0, 0};
    std::array<double, 3> const point{5, 10, 15};

    std::array<double, 3> const transformed_point{TransformPoint(tf.data(), point.data())};

    EXPECT_FLOAT_EQ(transformed_point[0], -10.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 5.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 15.0);
}
