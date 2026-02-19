#include "database/sensor_data_interface.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "testing_utilities/constants.hpp"
#include "testing_utilities/temporary_file.hpp"
#include "types/sensor_types.hpp"

#include "sqlite3_helpers.hpp"

using namespace reprojection;
using PoseType = database::PoseType;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(DatabaseSensorDataInterface, TestAddCameraPoseData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t const timestamp_ns{0};
    Frames const data{{timestamp_ns, {Array6d::Zero()}}};
    std::string_view const sensor_name{"/cam/retro/123"};

    // Fails foreign key constraint because there is no corresponding extracted_targets table entry yet
    EXPECT_THROW(database::AddCameraPoseData(data, PoseType::Initial, sensor_name, db), std::runtime_error);
    EXPECT_THROW(database::AddCameraPoseData(data, PoseType::Optimized, sensor_name, db), std::runtime_error);

    // Now we add an image and extracted target with matching sensor name and timestamp (i.e. the foreign key
    // constraint) and now we can add the initial camera pose no problem :)
    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db);
    EXPECT_NO_THROW(database::AddCameraPoseData(data, PoseType::Initial, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddSplinePoseData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    database::SplinePoses const data{{0, Array6d::Zero()}};
    std::string_view const sensor_name{"/cam/retro/123"};

    EXPECT_NO_THROW(database::AddSplinePoseData(data, PoseType::Initial, sensor_name, db));
    EXPECT_NO_THROW(database::AddSplinePoseData(data, PoseType::Optimized, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddReprojectionError) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t const timestamp_ns{0};
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};
    std::string_view const sensor_name{"/cam/retro/123"};

    // Fails foreign key constraint because there is no corresponding camera_poses table entry yet
    EXPECT_THROW(database::AddReprojectionError(data, PoseType::Initial, sensor_name, db), std::runtime_error);
    EXPECT_THROW(database::AddReprojectionError(data, PoseType::Optimized, sensor_name, db), std::runtime_error);

    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db);

    Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
    database::AddCameraPoseData(frames, PoseType::Initial, sensor_name, db);
    database::AddCameraPoseData(frames, PoseType::Optimized, sensor_name, db);

    EXPECT_NO_THROW(database::AddReprojectionError(data, PoseType::Initial, sensor_name, db));
    EXPECT_NO_THROW(database::AddReprojectionError(data, PoseType::Optimized, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddExtractedTargetData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t const timestamp_ns{0};
    std::string_view const sensor_name{"/cam/retro/123"};

    // Adding a target with no corresponding image database entry is invalid! Foreign key constraint :)
    EXPECT_THROW(AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db), std::runtime_error);

    database::AddImage(timestamp_ns, sensor_name, db);
    EXPECT_NO_THROW(AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestGetExtractedTargetData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t timestamp_ns{0};
    std::string_view const sensor_name{"/cam/retro/123"};
    ExtractedTarget const target{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                        MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                                 {{5, 6}, {2, 3}, {650, 600}}};

    // Add three targets - due to foreign key relationship we need add an image before we add the target
    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, target}, sensor_name, db);
    timestamp_ns = 1;
    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, target}, sensor_name, db);
    timestamp_ns = 2;
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
    std::vector<ImuMeasurement> const data{{0, {}, {}},  //
                                           {1, {}, {}},
                                           {2, {}, {}}};

    // NOTE(Jack): We use the local scopes here so that we can have a create/read/write and a const read only
    // database instance in the same test using the same file.
    TemporaryFile const temp_file{".db3"};
    {
        // Create and write
        auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

        for (auto const& measurement_i : data) {
            bool const success_i{database::AddImuData(measurement_i, sensor_name, db)};
            EXPECT_TRUE(success_i);
        }
    }
    {
        // Const read only
        auto const db{std::make_shared<database::CalibrationDatabase const>(temp_file.Path(), false, true)};

        auto const loaded_data{database::GetImuData(db, sensor_name)};
        EXPECT_EQ(std::size(loaded_data), std::size(data));
    }
}

TEST(DatabaseSensorDataInterface, TestAddImuData) {
    TemporaryFile const temp_file{".db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true)};

    std::string_view sensor_name_1{"/imu/polaris/123"};
    bool success{database::AddImuData({0, {}, {}}, sensor_name_1, db)};
    EXPECT_TRUE(success);
    success = database::AddImuData({1, {}, {}}, sensor_name_1, db);
    EXPECT_TRUE(success);

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary
    // key (timestamp_ns, sensor_name) so it is not a duplicate
    std::string_view sensor_name_2{"/imu/polaris/456"};
    success = database::AddImuData({0, {}, {}}, sensor_name_2, db);
    EXPECT_TRUE(success);

    // Add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW((void)database::AddImuData({0, {}, {}}, sensor_name_2, db), std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestGetImuData) {
    TemporaryFile const temp_file{".db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true)};

    // Data from imu 123
    std::string_view sensor_name_1{"/imu/polaris/123"};
    (void)database::AddImuData({5, {1, 2, 3}, {4, 5, 6}}, sensor_name_1, db);
    (void)database::AddImuData({10, {}, {}}, sensor_name_1, db);
    (void)database::AddImuData({15, {}, {}}, sensor_name_1, db);
    // Data from imu 456
    std::string_view sensor_name_2{"/imu/polaris/456"};
    (void)database::AddImuData({10, {}, {}}, sensor_name_2, db);
    (void)database::AddImuData({20, {}, {}}, sensor_name_2, db);

    auto const imu_1_data{database::GetImuData(db, sensor_name_1)};
    EXPECT_EQ(std::size(imu_1_data), 3);

    // Check the values of the first element to make sure the callback lambda reading logic is correct
    ImuMeasurement const sample{imu_1_data.at(5)};
    EXPECT_EQ(sample.timestamp_ns, 5);
    EXPECT_EQ(sample.angular_velocity[0], 1);
    EXPECT_EQ(sample.angular_velocity[1], 2);
    EXPECT_EQ(sample.angular_velocity[2], 3);
    EXPECT_EQ(sample.linear_acceleration[0], 4);
    EXPECT_EQ(sample.linear_acceleration[1], 5);
    EXPECT_EQ(sample.linear_acceleration[2], 6);

    auto const imu_2_data{database::GetImuData(db, sensor_name_2)};
    EXPECT_EQ(std::size(imu_2_data), 2);

    // If the sensor is not present we simply get an empty set back, this is not an error
    auto const unknown_sensor_data{database::GetImuData(db, "/imu/polaris/unknown")};
    EXPECT_EQ(std::size(unknown_sensor_data), 0);
}
