#include "testing_mocks/imu_data_generator.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(TestingMocksImuDataGenerator, TestGenerateImuData) {
    ImuMeasurements const data{testing_mocks::GenerateImuData(100, 1e9)};
    EXPECT_EQ(std::size(data), 100);

    // TODO(Jack): Should we also heurstically test some random measurements? Or at least their timestamps?
}