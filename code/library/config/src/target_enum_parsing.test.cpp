#include "target_enum_parsing.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigTargetEnumParsing, TestStringToTargetTypeEnum) {
    TargetType type{config::StringToTargetTypeEnum("checkerboard")};
    EXPECT_EQ(type, TargetType::Checkerboard);
    type = config::StringToTargetTypeEnum("circle_grid");
    EXPECT_EQ(type, TargetType::CircleGrid);
    type = config::StringToTargetTypeEnum("april_grid3");
    EXPECT_EQ(type, TargetType::AprilGrid3);

    EXPECT_THROW(config::StringToTargetTypeEnum("nonexistent_target_type"), std::runtime_error);
}