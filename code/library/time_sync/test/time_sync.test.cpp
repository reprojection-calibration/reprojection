#include "time_sync/time_sync.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(TimeSync, TestFindClosest) {
    // Empty container throws.
    EXPECT_THROW(time_sync::FindClosest({}, 0), std::runtime_error);

    std::set<uint64_t> const data{100, 110};

    auto result{time_sync::FindClosest(data, 0)};  // Below.
    EXPECT_EQ(*result, 100);
    result = time_sync::FindClosest(data, 200);  // Above.
    EXPECT_EQ(*result, 110);
    result = time_sync::FindClosest(data, 101);  // In between.
    EXPECT_EQ(*result, 100);
    result = time_sync::FindClosest(data, 105);  // Split the middle "rounds down".
    EXPECT_EQ(*result, 100);
}

TEST(TimeSync, TestIsWithinThreshold) {
    EXPECT_TRUE(time_sync::IsWithinThreshold(100, 110, 15));
    EXPECT_TRUE(time_sync::IsWithinThreshold(100, 100, 15));
    EXPECT_TRUE(time_sync::IsWithinThreshold(100, 115, 15));
    EXPECT_TRUE(time_sync::IsWithinThreshold(115, 100, 15));
    EXPECT_TRUE(time_sync::IsWithinThreshold(100, 100, 0));  // Exact sync

    EXPECT_FALSE(time_sync::IsWithinThreshold(100, 116, 15));
    EXPECT_FALSE(time_sync::IsWithinThreshold(100, 101, 0));
}