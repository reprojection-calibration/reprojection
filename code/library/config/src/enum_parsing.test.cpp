#include "enum_parsing.hpp"

#include <gtest/gtest.h>

#include "ceres_enums.hpp"

namespace reprojection::config {}

using namespace reprojection;

TEST(ConfigEnumParsing, TestXXXX) {
    ceres::MinimizerType result{config::ParseEnum("TRUST_REGION", config::MinimizerTypeMap)};
    EXPECT_EQ(result, ceres::TRUST_REGION);

    result = config::ParseEnum("LINE_SEARCH", config::MinimizerTypeMap);
    EXPECT_EQ(result, ceres::LINE_SEARCH);

    EXPECT_THROW(config::ParseEnum("BAD_METHOD", config::MinimizerTypeMap), std::runtime_error);
}