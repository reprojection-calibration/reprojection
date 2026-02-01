#include "testing_mocks/imu_data_generator.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(TestingMocksImuDataGenerator, TestGenerateImuData) {
    testing_mocks::ImuData const data{testing_mocks::GenerateImuData(100)};

    uint64_t gt_timestamp_ns{0};
    for (auto const& [timestamp_ns, _] : data) {
        // NOTE(Jack): We allow a one nanosecond tolerance to account for rounding/casting errors.
        EXPECT_NEAR(timestamp_ns, gt_timestamp_ns, 1);

        gt_timestamp_ns += 1970000;
    }
    EXPECT_NEAR(gt_timestamp_ns, 195030000 + 1970000, 1);
}