#include "database/calibration_database.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(DatabaseCalibrationDatabase, TestCreate) {
    TemporaryFile const temp_file{".db3"};

    // Cannot create a database when read_only is true - creating a database requires writing to it!
    EXPECT_THROW(database::CalibrationDatabase(temp_file.Path(), true, true), std::runtime_error);

    // Cannot open a non-existent database
    EXPECT_THROW(database::CalibrationDatabase(temp_file.Path(), false), std::runtime_error);

    // Create a database (found on the filesystem) and then check that we can open it
    database::CalibrationDatabase(temp_file.Path(), true);
    EXPECT_NO_THROW(database::CalibrationDatabase(temp_file.Path(), false));
}

TEST(DatabaseCalibrationDatabase, TestReadWrite) {
    TemporaryFile const temp_file{".db3"};
    database::CalibrationDatabase(temp_file.Path(), true);

    EXPECT_NO_THROW(database::CalibrationDatabase(temp_file.Path(), false, false));
}

TEST(DatabaseCalibrationDatabase, TestReadOnly) {
    TemporaryFile const temp_file{".db3"};
    database::CalibrationDatabase(temp_file.Path(), true);

    EXPECT_NO_THROW(database::CalibrationDatabase(temp_file.Path(), false, true));
}