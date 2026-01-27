#include "projection_functions/image_bounds.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(ProjectionFunctionsImageBounds, TestImageBounds) {
    ImageBounds const bounds{testing_utilities::image_bounds};

    EXPECT_TRUE(projection_functions::InBounds(bounds, 0.0, 0.0));
    EXPECT_TRUE(projection_functions::InBounds(bounds, 0.0, 479.999));
    EXPECT_TRUE(projection_functions::InBounds(bounds, 719.999, 0.0));

    EXPECT_FALSE(projection_functions::InBounds(bounds, -1e3, -1e3));
    EXPECT_FALSE(projection_functions::InBounds(bounds, 0.0, 480.0));
    EXPECT_FALSE(projection_functions::InBounds(bounds, 720.0, 0.0));
}

TEST(ProjectionFunctionsImageBounds, TestImageBoundsNegative) {
    ImageBounds const bounds{testing_utilities::unit_image_bounds};

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
