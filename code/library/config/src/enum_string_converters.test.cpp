#include "enum_string_converters.hpp"

#include <ceres/types.h>
#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigTargetEnumParsing, TestStringToTargetTypeEnum) {
    TargetType type{config::ToTargetType("checkerboard")};
    EXPECT_EQ(type, TargetType::Checkerboard);
    type = config::ToTargetType("circle_grid");
    EXPECT_EQ(type, TargetType::CircleGrid);
    type = config::ToTargetType("april_grid3");
    EXPECT_EQ(type, TargetType::AprilGrid3);

    EXPECT_THROW(config::ToTargetType("nonexistent_target_type"), std::runtime_error);
}

TEST(ConfigEnumParsing, TestCeresEnumToString) {
    auto result = config::StringToCeresEnum<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
        "STEEPEST_DESCENT");
    EXPECT_EQ(result, ceres::STEEPEST_DESCENT);

    result = config::StringToCeresEnum<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
        "NONLINEAR_CONJUGATE_GRADIENT");
    EXPECT_EQ(result, ceres::NONLINEAR_CONJUGATE_GRADIENT);

    // Need the extra pair of parenthesis due to https://github.com/google/googletest/issues/3803
    EXPECT_THROW((config::StringToCeresEnum<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
                     "DOES_NOT_EXIST")),
                 std::runtime_error);
}