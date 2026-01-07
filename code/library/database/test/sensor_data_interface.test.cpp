#include "database/sensor_data_interface.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "types/sensor_types.hpp"

using namespace reprojection;

// Test fixture used to facilitate isolated filesystem state. This is useful when testing database creation to prevent
// artefacts from previous or parallel testing interfering with the system currently under test.
class TempFolder : public ::testing::Test {
   protected:
    void SetUp() override { std::filesystem::create_directories(database_path_); }

    void TearDown() override { std::filesystem::remove_all(database_path_); }

    std::string database_path_{"sandbox"};
};

TEST_F(TempFolder, TestAddPoseData) {
    std::string const record_path{database_path_ + "/record_ddd.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    Vector6d const pose{0, 1, 2, 3, 4, 5};

    // Fails foreign key constraint because there is no corresponding extracted_targets table entry yet
    EXPECT_FALSE(database::AddPoseData({{{0, "/cam/retro/123"}, pose}}, database::PoseTable::Camera,
                                       database::PoseType::Initial, db));
    EXPECT_FALSE(database::AddPoseData({{{0, "/cam/retro/123"}, pose}}, database::PoseTable::Camera,
                                       database::PoseType::Optimized, db));

    // Now we add an image and extracted target with matching sensor name and timestamp (i.e. the foreign key
    // constraint) and now we can add the initial camera pose no problem :)
    FrameHeader const header{0, "/cam/retro/123"};
    (void)database::AddImage(header, db);
    (void)AddExtractedTargetData({header, {}}, db);
    EXPECT_TRUE(database::AddPoseData({{header, pose}}, database::PoseTable::Camera, database::PoseType::Initial, db));

    // The external poses table has no foreign key constraint because external poses are not restricted to the image
    // frames. Therefore, we can add a pose here directly.
    EXPECT_TRUE(
        database::AddPoseData({{header, pose}}, database::PoseTable::External, database::PoseType::GroundTruth, db));
}

TEST_F(TempFolder, TestAddExtractedTargetData) {
    std::string const record_path{database_path_ + "/record_qqq.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    ExtractedTarget const bundle{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                        MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                                 {{5, 6}, {2, 3}, {650, 600}}};

    FrameHeader const header{0, "/cam/retro/123"};
    (void)database::AddImage(header, db);

    EXPECT_TRUE(AddExtractedTargetData({header, bundle}, db));
}

TEST_F(TempFolder, TestGetExtractedTargetData) {
    std::string const record_path{database_path_ + "/record_qqq.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    ExtractedTarget const target{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                        MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                                 {{5, 6}, {2, 3}, {650, 600}}};

    // Due to foreign key relationship we need add an image before we add the extracted target
    FrameHeader header{0, "/cam/retro/123"};
    (void)database::AddImage(header, db);
    (void)AddExtractedTargetData({header, target}, db);
    header.timestamp_ns = 1;
    (void)database::AddImage(header, db);
    (void)AddExtractedTargetData({header, target}, db);
    header.timestamp_ns = 2;
    (void)database::AddImage(header, db);
    (void)AddExtractedTargetData({header, target}, db);

    auto const data{database::GetExtractedTargetData(db, "/cam/retro/123")};
    ASSERT_TRUE(data.has_value());
    EXPECT_EQ(std::size(data.value()), 3);

    int timestamp{0};
    for (auto const& data_i : data.value()) {
        EXPECT_EQ(data_i.header.timestamp_ns, timestamp);
        timestamp = timestamp + 1;
        EXPECT_TRUE(data_i.target.bundle.pixels.isApprox(target.bundle.pixels));
        EXPECT_TRUE(data_i.target.bundle.points.isApprox(target.bundle.points));
        EXPECT_TRUE(data_i.target.indices.isApprox(target.indices));
    }
}

TEST_F(TempFolder, TestFullImuAddGetCycle) {
    // TODO(Jack): Should we use this test data for all the IMU tests?
    std::map<std::string, std::set<ImuStamped>> const imu_data{
        {"/imu/polaris/123",
         {{{0, "/imu/polaris/123"}, {0, 0, 0}, {1, 1, 1}},
          {{1, "/imu/polaris/123"}, {2, 2, 2}, {3, 3, 3}},
          {{3, "/imu/polaris/123"}, {4, 4, 4}, {5, 5, 5}}}},
        {"/imu/polaris/456",
         {{{1, "/imu/polaris/456"}, {0, 0, 0}, {-1, -1, -1}}, {{2, "/imu/polaris/456"}, {-2, -2, -2}, {-3, -3, -3}}}}};

    std::string const record_path{database_path_ + "/record_aaa.db3"};
    // NOTE(Jack): We use the local scopes here so that we can have a create/read/write and a const read only database
    // instance in the same test.
    // TODO(Jack): Is it legal to open two connection to the samedatabase?
    {
        auto const db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};  // create and write
        for (auto const& [sensor, measurements] : imu_data) {
            for (auto const& measurement_i : measurements) {
                bool const success_i{database::AddImuData(measurement_i, db)};
                EXPECT_TRUE(success_i);
            }
        }
    }
    {
        auto const db{
            std::make_shared<database::CalibrationDatabase const>(record_path, false, true)};  // const read only
        for (auto const& [sensor, measurements] : imu_data) {
            auto const imu_i_data{database::GetImuData(db, sensor)};
            ASSERT_TRUE(imu_i_data.has_value());
            EXPECT_EQ(std::size(imu_i_data.value()), std::size(measurements));
        }
    }
}

TEST_F(TempFolder, TestAddImuData) {
    std::string const record_path{database_path_ + "/record_hhh.db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(record_path, true)};

    bool success{database::AddImuData({{0, "/imu/polaris/123"}, {}, {}}, db)};
    EXPECT_TRUE(success);
    success = database::AddImuData({{1, "/imu/polaris/123"}, {}, {}}, db);
    EXPECT_TRUE(success);

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary key
    // (timestamp_ns, sensor_name) so it is not a duplicate
    success = database::AddImuData({{0, "/imu/polaris/456"}, {}, {}}, db);
    EXPECT_TRUE(success);

    // Add a repeated record - this is not successful because the primary key must always be unique!
    testing::internal::CaptureStderr();  // WARN USING INTERNAL GTEST API!
    success = database::AddImuData({{0, "/imu/polaris/456"}, {}, {}}, db);
    EXPECT_FALSE(success);

    // Check that the expected error message is sent to cerr
    std::string const error_message{testing::internal::GetCapturedStderr()};  // WARN USING INTERNAL GTEST API!
    EXPECT_EQ(
        error_message,
        "AddImuData() sqlite3_step() failed: UNIQUE constraint failed: imu_data.timestamp_ns, imu_data.sensor_name\n");
}

TEST_F(TempFolder, TestGetImuData) {
    std::string const record_path{database_path_ + "/record_aaa.db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(record_path, true)};

    // Data from imu 123
    (void)database::AddImuData({{5, "/imu/polaris/123"}, {1, 2, 3}, {4, 5, 6}}, db);
    (void)database::AddImuData({{10, "/imu/polaris/123"}, {}, {}}, db);
    (void)database::AddImuData({{15, "/imu/polaris/123"}, {}, {}}, db);
    // Data from imu 456
    (void)database::AddImuData({{10, "/imu/polaris/456"}, {}, {}}, db);
    (void)database::AddImuData({{20, "/imu/polaris/456"}, {}, {}}, db);

    auto const imu_123_data{database::GetImuData(db, "/imu/polaris/123")};
    ASSERT_TRUE(imu_123_data.has_value());
    EXPECT_EQ(std::size(imu_123_data.value()), 3);

    // Check the values of the first element to make sure the callback lambda reading logic is correct
    ImuStamped const sample{*std::cbegin(imu_123_data.value())};
    EXPECT_EQ(sample.header.timestamp_ns, 5);
    EXPECT_EQ(sample.angular_velocity[0], 1);
    EXPECT_EQ(sample.angular_velocity[1], 2);
    EXPECT_EQ(sample.angular_velocity[2], 3);
    EXPECT_EQ(sample.linear_acceleration[0], 4);
    EXPECT_EQ(sample.linear_acceleration[1], 5);
    EXPECT_EQ(sample.linear_acceleration[2], 6);

    auto const imu_456_data{database::GetImuData(db, "/imu/polaris/456")};
    ASSERT_TRUE(imu_456_data.has_value());
    EXPECT_EQ(std::size(imu_456_data.value()), 2);

    // If the sensor is not present we simply get an empty set back, this is not an error
    auto const unknown_sensor{database::GetImuData(db, "/imu/polaris/unknown")};
    ASSERT_TRUE(unknown_sensor.has_value());
    EXPECT_EQ(std::size(unknown_sensor.value()), 0);
}
