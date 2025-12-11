#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(FeatureExtractionUtilities, TestAlternatingSum) {
    double const sum_1{feature_extraction::AlternatingSum(2, 0.5, 0.2)};
    EXPECT_EQ(sum_1, 0.5 + 0.2);

    double const sum_2{feature_extraction::AlternatingSum(3, 0.5, 0.2)};
    EXPECT_EQ(sum_2, 0.5 + 0.2 + 0.5);

    double const sum_null{feature_extraction::AlternatingSum(0, 0.5, 0.2)};
    EXPECT_EQ(sum_null, 0);
}