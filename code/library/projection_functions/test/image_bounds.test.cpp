#include "projection_functions/image_bounds.hpp"

#include <gtest/gtest.h>

#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(ProjectionFunctionsImageBounds, TestImageBounds) {
    ImageBounds const bounds{-1, 1, -1, 1};

    EXPECT_TRUE(projection_functions::InBounds(bounds, 0, 0));
    EXPECT_TRUE(projection_functions::InBounds(bounds, 0.99, 0.99));
    EXPECT_TRUE(projection_functions::InBounds(bounds, -0.99, 0.99));
    EXPECT_TRUE(projection_functions::InBounds(bounds, 0.99, -0.99));
    EXPECT_TRUE(projection_functions::InBounds(bounds, -0.99, -0.99));

    EXPECT_FALSE(projection_functions::InBounds(bounds, 1.01, 1.01));
    EXPECT_FALSE(projection_functions::InBounds(bounds, -1.01, 1.01));
    EXPECT_FALSE(projection_functions::InBounds(bounds, 1.01, -1.01));
    EXPECT_FALSE(projection_functions::InBounds(bounds, -1.01, -1.01));

    EXPECT_FALSE(projection_functions::InBounds(bounds, 0.0, 1.01));
    EXPECT_FALSE(projection_functions::InBounds(bounds, 1.01, 0.0));
}
