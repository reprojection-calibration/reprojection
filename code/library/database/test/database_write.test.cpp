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

TEST_F(CameraDatabaseFixture, TestWriteToDbPoseData) {
    // Throws because the foreign key constraints are not met yet.
    EXPECT_THROW(InsertPose(CalibrationStep::PoseInitialization), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImage();
    InsertTarget();
    InsertStep(CalibrationStep::PoseInitialization);

    // Passes with no problem.
    EXPECT_NO_THROW(InsertPose(CalibrationStep::PoseInitialization));
}

TEST_F(CameraDatabaseFixture, TestInsertReprojectionErrors) {
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};

    // Throws because the foreign key constraints are not met yet.
    EXPECT_THROW(database::InsertReprojectionErrors(db, sensor_name, CalibrationStep::PoseInitialization, data),
                 std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImage();
    InsertTarget();
    InsertStep(CalibrationStep::PoseInitialization);
    InsertPose(CalibrationStep::PoseInitialization);

    EXPECT_NO_THROW(database::InsertReprojectionErrors(db, sensor_name, CalibrationStep::PoseInitialization, data));
}

class ImuDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::InsertEntity(db, sensor_name, Entity::Imu);
    }

    void AddStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        database::InsertStep(db, sensor_name, step_name, cache_key);
    }

    void AddImuData() const {
        database::InsertImuData(db, sensor_name, ImuMeasurements{{timestamp_ns, {Vector3d::Zero(), Vector3d::Zero()}}});
    }

    void AddImuError() const {
        database::InsertImuErrors(db, sensor_name, CalibrationStep::ExtrinsicInitialization,
                                  ImuErrors{{timestamp_ns, {Vector3d::Zero(), Vector3d::Zero()}}});
    }

    SqlitePtr db{nullptr};
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/imu/polaris/123"};
};

TEST_F(ImuDatabaseFixture, TestInsertImuData) {
    EXPECT_NO_THROW(AddImuData());

    // Duplicate entry not allowed!
    EXPECT_THROW(AddImuData(), std::runtime_error);
}

TEST_F(ImuDatabaseFixture, TestInsertImuErrors) {
    // Foreign key requirements not met
    EXPECT_THROW(AddImuError(), std::runtime_error);

    // Satisfy foreign key requirements and then it works fine :)
    AddImuData();
    AddStep(CalibrationStep::ExtrinsicInitialization);
    EXPECT_NO_THROW(AddImuError());

    // Duplicate entry not allowed!
    EXPECT_THROW(AddImuError(), std::runtime_error);
}

class ExtrinsicDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::InsertEntity(db, camera_name, Entity::Camera);
        database::InsertEntity(db, extrinsic_entity_id, Entity::Extrinsic);
    }

    SqlitePtr db{nullptr};
    // The spline stuff gets associated with the camera it comes from and the actual extrinsic tf (including gravity,
    // but not sure if that makes sense long term) get associated with an extrinsic specific entity id.
    std::string camera_name{"/cam/retro/123"};
    std::string extrinsic_entity_id{"tf_imu_co"};
};

TEST_F(ExtrinsicDatabaseFixture, TestInsertControlPoints) {
    spline::Matrix2NXd const control_points{spline::Matrix2NXd::Random(6, 10)};

    EXPECT_THROW(database::InsertControlPoints(db, camera_name, CalibrationStep::SplineInitialization, control_points),
                 std::runtime_error);

    // Satisfy foreign key requirement
    database::InsertStep(db, camera_name, CalibrationStep::SplineInitialization, "");

    EXPECT_NO_THROW(
        database::InsertControlPoints(db, camera_name, CalibrationStep::SplineInitialization, control_points));

    // Duplicate entry throws
    EXPECT_THROW(database::InsertControlPoints(db, camera_name, CalibrationStep::SplineInitialization, control_points),
                 std::runtime_error);
}

TEST_F(ExtrinsicDatabaseFixture, TestInsertTimeHandler) {
    spline::TimeHandler const time_handler{100, 200};

    EXPECT_THROW(database::InsertTimeHandler(db, camera_name, CalibrationStep::SplineInitialization, time_handler),
                 std::runtime_error);

    // Satisfy foreign key requirement
    database::InsertStep(db, camera_name, CalibrationStep::SplineInitialization, "");

    EXPECT_NO_THROW(database::InsertTimeHandler(db, camera_name, CalibrationStep::SplineInitialization, time_handler));

    // Duplicate entry throws
    EXPECT_THROW(database::InsertTimeHandler(db, camera_name, CalibrationStep::SplineInitialization, time_handler),
                 std::runtime_error);
}

// TODO(Jack): Should the gravity really be associated with the extrinsic entity?
TEST_F(ExtrinsicDatabaseFixture, TestInsertGravity) {
    Array3d const gravity_w{0, 1, 2};

    EXPECT_THROW(database::InsertGravity(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, gravity_w),
                 std::runtime_error);

    // Satisfy foreign key requirement
    database::InsertStep(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, "");

    EXPECT_NO_THROW(
        database::InsertGravity(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, gravity_w));

    // Duplicate entry throws
    EXPECT_THROW(database::InsertGravity(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, gravity_w),
                 std::runtime_error);
}

TEST_F(ExtrinsicDatabaseFixture, TestInsertExtrinsic) {
    Array6d const tf_imu_co{0, 1, 2, 3, 4, 5};

    EXPECT_THROW(
        database::InsertExtrinsic(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, tf_imu_co),
        std::runtime_error);

    // Satisfy foreign key requirement
    database::InsertStep(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, "");

    EXPECT_NO_THROW(
        database::InsertExtrinsic(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, tf_imu_co));

    // Duplicate entry throws
    EXPECT_THROW(
        database::InsertExtrinsic(db, extrinsic_entity_id, CalibrationStep::ExtrinsicInitialization, tf_imu_co),
        std::runtime_error);
}