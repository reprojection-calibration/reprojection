#include "time_synchronization.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(CalibrationTimeSynchronization, TestFindClosest) {
    // Empty container throws.
    EXPECT_THROW(calibration::FindClosest({}, 0), std::runtime_error);

    std::set<uint64_t> const data{100, 110};

    auto result{calibration::FindClosest(data, 0)};  // Below.
    EXPECT_EQ(*result, 100);
    result = calibration::FindClosest(data, 200);  // Above.
    EXPECT_EQ(*result, 110);
    result = calibration::FindClosest(data, 101);  // In between.
    EXPECT_EQ(*result, 100);
    result = calibration::FindClosest(data, 105);  // Split the middle "rounds down".
    EXPECT_EQ(*result, 100);
}

TEST(CalibrationTimeSynchronization, TestIsWithinThreshold) {
    EXPECT_TRUE(calibration::IsWithinThreshold(100, 110, 15));
    EXPECT_TRUE(calibration::IsWithinThreshold(100, 100, 15));
    EXPECT_TRUE(calibration::IsWithinThreshold(100, 115, 15));
    EXPECT_TRUE(calibration::IsWithinThreshold(115, 100, 15));
    EXPECT_TRUE(calibration::IsWithinThreshold(100, 100, 0));  // Exact sync

    EXPECT_FALSE(calibration::IsWithinThreshold(100, 116, 15));
    EXPECT_FALSE(calibration::IsWithinThreshold(100, 101, 0));
}