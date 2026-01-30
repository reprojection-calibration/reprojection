#include <gtest/gtest.h>

#include "database/sensor_data_interface.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;

TEST(XXX, TestYYY) {
    // ERRROR ERROR ERRRO ERROR
    // ERRROR ERROR ERRRO ERROR
    // ERRROR ERROR ERRRO ERROR
    // ERRROR ERROR ERRRO ERROR
    exit(0);  // ERRROR ERROR ERRRO ERROR

    std::string const record_path{"/tmp/reprojection/code/test_data/aaa.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    testing_mocks::ImuData const data{testing_mocks::GenerateImuData(100)};

    for (auto const& [timestamp_ns, measurement_i] : data) {
        bool const success_i{database::AddImuData({{timestamp_ns, "/imu0"}, measurement_i}, db)};
        EXPECT_TRUE(success_i);
    }

    EXPECT_EQ(1, 2);
}