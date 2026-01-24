#include "enum_parsing.hpp"

#include <gtest/gtest.h>

#include "ceres_enums.hpp"

namespace reprojection::config {}  // namespace reprojection::config

using namespace reprojection;

// REMOVE HARDCODED MAPS!

TEST(ConfigEnumParsing, TestCeresEnumToString) {
    auto result = config::CeresEnumToString<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
        "STEEPEST_DESCENT");
    EXPECT_EQ(result, ceres::STEEPEST_DESCENT);

    result = config::CeresEnumToString<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
        "NONLINEAR_CONJUGATE_GRADIENT");
    EXPECT_EQ(result, ceres::NONLINEAR_CONJUGATE_GRADIENT);

    // Need the extra pair of parenthesis due to https://github.com/google/googletest/issues/3803
    EXPECT_THROW((config::CeresEnumToString<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
                     "DOES_NOT_EXIST")),
                 std::runtime_error);
}