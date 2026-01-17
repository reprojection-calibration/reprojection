#include "projection_functions/image_bounds.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ProjectionFunctionsImageBounds, TestImageBounds) {
    projection_functions::ImageBounds const image_bounds{{-1, 1, -1, 1}};

    EXPECT_TRUE(image_bounds.InBounds(0, 0));
    EXPECT_TRUE(image_bounds.InBounds(0.99, 0.99));
    EXPECT_TRUE(image_bounds.InBounds(-0.99, 0.99));
    EXPECT_TRUE(image_bounds.InBounds(0.99, -0.99));
    EXPECT_TRUE(image_bounds.InBounds(-0.99, -0.99));

    EXPECT_FALSE(image_bounds.InBounds(1.01, 1.01));
    EXPECT_FALSE(image_bounds.InBounds(-1.01, 1.01));
    EXPECT_FALSE(image_bounds.InBounds(1.01, -1.01));
    EXPECT_FALSE(image_bounds.InBounds(-1.01, -1.01));

    EXPECT_FALSE(image_bounds.InBounds(0.0, 1.01));
    EXPECT_FALSE(image_bounds.InBounds(1.01, 0.0));
}
