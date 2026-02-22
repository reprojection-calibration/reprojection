#include "database/sensor_data_interface_getters.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "testing_utilities/constants.hpp"
#include "testing_utilities/temporary_file.hpp"
#include "types/sensor_data_types.hpp"

using namespace reprojection;
using PoseType = database::PoseType;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(DatabaseSensorDataInterface, TestGetExtractedTargetData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t timestamp_ns{0};
    std::string const sensor_name{"/cam/retro/123"};
    ExtractedTarget const target{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                        MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                                 {{5, 6}, {2, 3}, {650, 600}}};

    // Add three targets - due to foreign key relationship we need add an image before we add the target
    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, target}, sensor_name, db);
    timestamp_ns += 1;
    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, target}, sensor_name, db);
    timestamp_ns += 1;
    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, target}, sensor_name, db);

    CameraMeasurements const loaded_data{database::GetExtractedTargetData(db, sensor_name)};
    EXPECT_EQ(std::size(loaded_data), 3);

    int test_timestamp{0};
    for (auto const& [timestamp_ns_i, target_i] : loaded_data) {
        EXPECT_EQ(timestamp_ns_i, test_timestamp);
        test_timestamp += 1;

        EXPECT_TRUE(target_i.bundle.pixels.isApprox(target.bundle.pixels));
        EXPECT_TRUE(target_i.bundle.points.isApprox(target.bundle.points));
        EXPECT_TRUE(target_i.indices.isApprox(target.indices));
    }
}

TEST(DatabaseSensorDataInterface, TestFullImuAddGetCycle) {
    std::string_view sensor_name{"/imu/polaris/123"};
    ImuMeasurements const data{{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                               {1, {Vector3d::Zero(), Vector3d::Zero()}},
                               {2, {Vector3d::Zero(), Vector3d::Zero()}}};

    // NOTE(Jack): We use the local scopes here so that we can have a create/read/write and a const read only
    // database instance in the same test using the same file.
    TemporaryFile const temp_file{".db3"};
    {
        // Create and write
        auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

        EXPECT_NO_THROW(database::AddImuData(data, sensor_name, db));
    }
    {
        // Const read only
        auto const db{std::make_shared<database::CalibrationDatabase const>(temp_file.Path(), false, true)};

        auto const loaded_data{database::GetImuData(db, sensor_name)};
        EXPECT_EQ(std::size(loaded_data), std::size(data));
    }
}

TEST(DatabaseSensorDataInterface, TestGetImuData) {
    TemporaryFile const temp_file{".db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true)};

    // Data from imu 123
    std::string_view sensor_name_1{"/imu/polaris/123"};
    database::AddImuData({{5, {{1, 2, 3}, {4, 5, 6}}},  //
                          {10, {Vector3d::Zero(), Vector3d::Zero()}},
                          {15, {Vector3d::Zero(), Vector3d::Zero()}}},
                         sensor_name_1, db);
    // Data from imu 456
    std::string_view sensor_name_2{"/imu/polaris/456"};
    database::AddImuData({{10, {Vector3d::Zero(), Vector3d::Zero()}},  //
                          {20, {Vector3d::Zero(), Vector3d::Zero()}}},
                         sensor_name_2, db);

    auto const imu_1_data{database::GetImuData(db, sensor_name_1)};
    EXPECT_EQ(std::size(imu_1_data), 3);

    // Check the values of the first element to make sure the callback lambda reading logic is correct
    ImuData const imu_data_i{imu_1_data.at(5)};
    EXPECT_EQ(imu_data_i.angular_velocity[0], 1);
    EXPECT_EQ(imu_data_i.angular_velocity[1], 2);
    EXPECT_EQ(imu_data_i.angular_velocity[2], 3);
    EXPECT_EQ(imu_data_i.linear_acceleration[0], 4);
    EXPECT_EQ(imu_data_i.linear_acceleration[1], 5);
    EXPECT_EQ(imu_data_i.linear_acceleration[2], 6);

    auto const imu_2_data{database::GetImuData(db, sensor_name_2)};
    EXPECT_EQ(std::size(imu_2_data), 2);

    // If the sensor is not present we simply get an empty set back, this is not an error
    auto const unknown_sensor_data{database::GetImuData(db, "/imu/polaris/unknown")};
    EXPECT_EQ(std::size(unknown_sensor_data), 0);
}
