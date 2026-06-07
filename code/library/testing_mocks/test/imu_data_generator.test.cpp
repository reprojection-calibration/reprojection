#include "testing_mocks/imu_data_generator.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(TestingMocksImuDataGenerator, TestGenerateImuData) {
    auto const [imu_data, trajectory]{testing_mocks::GenerateImuData(100, 1e9)};
    EXPECT_EQ(std::size(imu_data), 100);
    EXPECT_EQ(trajectory.Size(), 500);

    // TODO(Jack): Should we also heurstically test some random measurements? Or at least their timestamps?
}