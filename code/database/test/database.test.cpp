#include "database/database.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

using namespace reprojection;

// Test fixture used to facilitate isolated filesystem state. This is useful when testing database creation to prevent
// artefacts from previous or parallel testing interfering with the system currently under test.
class TempFolder : public ::testing::Test {
   protected:
    void SetUp() override { std::filesystem::create_directories(database_path_); }

    void TearDown() override { std::filesystem::remove_all(database_path_); }

    std::string database_path_{"sandbox"};
};

TEST_F(TempFolder, TestFullImuAddGetCycle) {
    // TODO(Jack): Should we use this test data for all the IMU tests?
    std::map<std::string, std::set<database::ImuData>> const imu_data{
        {"/imu/polaris/123", {{0, {0, 0, 0}, {1, 1, 1}}, {1, {2, 2, 2}, {3, 3, 3}}, {3, {4, 4, 4}, {5, 5, 5}}}},
        {"/imu/polaris/456", {{1, {0, 0, 0}, {-1, -1, -1}}, {2, {-2, -2, -2}, {-3, -3, -3}}}}};

    std::string const record_path{database_path_ + "/record_aaa.db3"};
    database::CalibrationDatabase db{record_path, true};
    for (auto const& [sensor, measurements] : imu_data) {
        for (auto const& measurement_i : measurements) {
            bool const success_i{db.AddImuData(sensor, measurement_i)};
            EXPECT_TRUE(success_i);
        }
    }

    for (auto const& [sensor, measurements] : imu_data) {
        auto const imu_i_data{db.GetImuData(sensor)};
        ASSERT_TRUE(imu_i_data.has_value());
        EXPECT_EQ(std::size(imu_i_data.value()), std::size(measurements));
    }
}

TEST_F(TempFolder, TestAddImuData) {
    std::string const record_path{database_path_ + "/record_hhh.db3"};
    database::CalibrationDatabase db{record_path, true};

    bool success{db.AddImuData("/imu/polaris/123", {0, {}, {}})};
    EXPECT_TRUE(success);
    success = db.AddImuData("/imu/polaris/123", {1, {}, {}});
    EXPECT_TRUE(success);

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary key
    // (timestamp_ns, sensor_name) so it is not a duplicate
    success = db.AddImuData("/imu/polaris/456", {0, {}, {}});
    EXPECT_TRUE(success);

    // Add a repeated record - this is not successful because the primary key must always be unique!
    testing::internal::CaptureStderr();  // WARN USING INTERNAL GTEST API!
    success = db.AddImuData("/imu/polaris/456", {0, {}, {}});
    EXPECT_FALSE(success);

    // Check that the expected error message is sent to cerr
    std::string const output{testing::internal::GetCapturedStderr()};  // WARN USING INTERNAL GTEST API!
    EXPECT_EQ(output, "SQL error: UNIQUE constraint failed: imu_data.timestamp_ns, imu_data.sensor_name\n");
}

TEST_F(TempFolder, TestGetImuData) {
    std::string const record_path{database_path_ + "/record_aaa.db3"};
    database::CalibrationDatabase db{record_path, true};

    // Data from imu 123
    (void)db.AddImuData("/imu/polaris/123", {0, {}, {}});
    (void)db.AddImuData("/imu/polaris/123", {1, {}, {}});
    (void)db.AddImuData("/imu/polaris/123", {2, {}, {}});
    // Data from imu 456
    (void)db.AddImuData("/imu/polaris/456", {10, {}, {}});
    (void)db.AddImuData("/imu/polaris/456", {20, {}, {}});

    auto const imu_123_data{db.GetImuData("/imu/polaris/123")};
    ASSERT_TRUE(imu_123_data.has_value());
    EXPECT_EQ(std::size(imu_123_data.value()), 3);

    auto const imu_456_data{db.GetImuData("/imu/polaris/456")};
    ASSERT_TRUE(imu_456_data.has_value());
    EXPECT_EQ(std::size(imu_456_data.value()), 2);
}

TEST_F(TempFolder, TestCreate) {
    std::string const record_path{database_path_ + "/record_xxx.db3"};

    // Cannot create a database when read_only is true - creating a database requires writing to it!
    EXPECT_THROW(database::CalibrationDatabase(record_path, true, true), std::runtime_error);

    // Cannot open a non-existent database
    EXPECT_THROW(database::CalibrationDatabase(record_path, false), std::runtime_error);

    // Create a database (found on the filesystem) and then check that we can open it
    database::CalibrationDatabase(record_path, true);
    EXPECT_NO_THROW(database::CalibrationDatabase(record_path, false));
}

TEST_F(TempFolder, TestReadWrite) {
    std::string const record_path{database_path_ + "/record_yyy.db3"};
    database::CalibrationDatabase(record_path, true);

    EXPECT_NO_THROW(database::CalibrationDatabase(record_path, false, false));
}

TEST_F(TempFolder, TestReadOnly) {
    std::string const record_path{database_path_ + "/record_zzz.db3"};
    database::CalibrationDatabase(record_path, true);

    EXPECT_NO_THROW(database::CalibrationDatabase(record_path, false, true));
}