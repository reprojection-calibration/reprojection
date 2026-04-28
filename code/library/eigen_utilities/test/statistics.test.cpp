#include "eigen_utilities/statistics.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(EigenUtiltiesStatistics, TestMedian) {
    ArrayXd data{Array3d{0.1, 2.3, 1.1}};
    double median{eigen_utilities::Median(data)};
    EXPECT_FLOAT_EQ(median, 1.1);

    data = Array4d{1, 3, 0, 2};
    median = eigen_utilities::Median(data);
    EXPECT_FLOAT_EQ(median, 1.5);
}
