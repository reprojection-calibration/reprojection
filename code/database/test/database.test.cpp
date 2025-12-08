#include "database/database.hpp"

#include <gtest/gtest.h>

#include <filesystem>
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

TEST_F(TempFolder, TestAddImuData) {
    std::string const record{database_path_ + "/record_hhh.db3"};
    database::CalibrationDatabase db{record, true};

    bool success{db.AddImuData("/imu/polaris/123", {0, {}, {}})};
    EXPECT_TRUE(success);
    success = db.AddImuData("/imu/polaris/123", {1, {}, {}});
    EXPECT_TRUE(success);

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary key
    // (timestamp_ns, sensor_name)
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

TEST_F(TempFolder, TestCreate) {
    std::string const record{database_path_ + "/record_xxx.db3"};

    // Cannot create a database when read_only is true - creating a database requires writing to it!
    EXPECT_THROW(database::CalibrationDatabase(record, true, true), std::runtime_error);

    // Cannot open a non-existent database
    EXPECT_THROW(database::CalibrationDatabase(record, false), std::runtime_error);

    // Create a database (found on the filesystem) and then check that we can open it
    database::CalibrationDatabase(record, true);
    EXPECT_NO_THROW(database::CalibrationDatabase(record, false));
}

TEST_F(TempFolder, TestReadWrite) {
    std::string const record{database_path_ + "/record_yyy.db3"};
    database::CalibrationDatabase(record, true);

    EXPECT_NO_THROW(database::CalibrationDatabase(record, false, false));
}

TEST_F(TempFolder, TestReadOnly) {
    std::string const record{database_path_ + "/record_zzz.db3"};
    database::CalibrationDatabase(record, true);

    EXPECT_NO_THROW(database::CalibrationDatabase(record, false, true));
}