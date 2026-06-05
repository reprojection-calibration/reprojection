#include "database/database_write.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

#include "database_test_fixtures.hpp"
#include "sqlite3_helpers.hpp"

using namespace reprojection;

// TODO(Jack): Refactor the tests that do not currently have a test fixture to either use the existing test fixture or
// add and imu test fixture and a extrinsic calibration test fixture to find a place for them.

// TODO(Jack): Should the gravity really be associated with the extrinsic entity?
TEST_F(ExtrinsicDatabaseFixture, TestInsertGravity) {
    Array3d const gravity_w{0, 1, 2};

    EXPECT_THROW(database::InsertGravity(db, extrinsic_id, CalibrationStep::ExtrinsicInitialization, gravity_w),
                 std::runtime_error);

    // Satisfy foreign key requirement
    database::InsertStep(db, extrinsic_id, CalibrationStep::ExtrinsicInitialization, "");

    EXPECT_NO_THROW(database::InsertGravity(db, extrinsic_id, CalibrationStep::ExtrinsicInitialization, gravity_w));

    // Duplicate entry throws
    EXPECT_THROW(database::InsertGravity(db, extrinsic_id, CalibrationStep::ExtrinsicInitialization, gravity_w),
                 std::runtime_error);
}
