#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection::feature_extraction;

TEST(FeatureExtractionUtilities, TestAlternatingSum) {
    double const sum_1{AlternatingSum(2, 0.5, 0.2)};
    EXPECT_EQ(sum_1, 0.5 + 0.2);

    double const sum_2{AlternatingSum(3, 0.5, 0.2)};
    EXPECT_EQ(sum_2, 0.5 + 0.2 + 0.5);

    double const sum_null{AlternatingSum(0, 0.5, 0.2)};
    EXPECT_EQ(sum_null, 0);
}