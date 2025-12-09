#include "database/calibration_database.hpp"

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