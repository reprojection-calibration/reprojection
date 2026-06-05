#include "database/database_write.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

#include "sqlite3_helpers.hpp"

using namespace reprojection;

class SensorDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override { db = database::OpenCalibrationDatabase(":memory:", true, false); }

    void AddEntity() const { database::InsertEntity(db, sensor_name, Entity::Camera); }

    void AddStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        database::InsertStep(db, sensor_name, step_name, cache_key);
    }

    void AddCamera() const {
        AddStep(CalibrationStep::CameraInfo);
        database::InsertCameraInfo(db, CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds});
    }

    void AddImage() const {
        AddStep(CalibrationStep::ImageLoading);
        database::InsertImages(db, sensor_name, EncodedImages{{timestamp_ns, {}}});
    }

    void AddTarget() const {
        AddStep(CalibrationStep::FeatureExtraction);
        database::InsertTargets(db, sensor_name, {{timestamp_ns, ExtractedTarget{Bundle{{}, {}}, {}}}});
    }

    void AddPose(CalibrationStep const step_name) const {
        Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
        database::InsertPoses(db, sensor_name, step_name, frames);
    }

    SqlitePtr db{nullptr};
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/cam/retro/123"};
};

TEST_F(SensorDatabaseFixture, TestInsertEntityInfo) {
    EXPECT_NO_THROW(AddEntity());
    EXPECT_THROW(AddEntity(), std::runtime_error);  // Duplicate entry not allowed!
}

TEST_F(SensorDatabaseFixture, TestInsertCameraInfo) {
    EXPECT_NO_THROW(AddCamera());
    EXPECT_THROW(AddCamera(), std::runtime_error);  // Duplicate entry not allowed!
}

// TODO(Jack): If we have foreign key constraints one day, like we will have to for the multi-target case, then we can
// add helpers like we have in the test fixture for other types, but for now we do not need to add a target info to the
// database for any other reason.
TEST_F(SensorDatabaseFixture, TestInsertTargetInfo) {
    database::InsertStep(db, sensor_name, CalibrationStep::TargetInfo, "");

    EXPECT_NO_THROW(database::InsertTargetInfo(db, sensor_name, TargetInfo{TargetType::Aprilgrid3, 8, 6, 0.1, false}));
    EXPECT_THROW(database::InsertTargetInfo(db, sensor_name, TargetInfo{TargetType::Aprilgrid3, 8, 6, 0.1, false}),
                 std::runtime_error);
}

TEST_F(SensorDatabaseFixture, TestInsertImages) {
    EXPECT_NO_THROW(AddImage());
    EXPECT_THROW(AddImage(), std::runtime_error);
}

TEST_F(SensorDatabaseFixture, TestInsertTargets) {
    EXPECT_THROW(database::InsertTargets(db, sensor_name, CameraMeasurements{{timestamp_ns, {}}}), std::runtime_error);

    AddImage();
    database::InsertStep(db, sensor_name, CalibrationStep::FeatureExtraction, "");

    EXPECT_NO_THROW(database::InsertTargets(db, sensor_name, CameraMeasurements{{timestamp_ns, {}}}));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbCalibrationStep) {
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization));
    EXPECT_NO_THROW(AddStep(CalibrationStep::BundleAdjustment));
    EXPECT_NO_THROW(AddStep(CalibrationStep::SplineInterpolation));
    EXPECT_NO_THROW(AddStep(CalibrationStep::SplineNonlinearRefinement));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbCalibrationStepUpsert) {
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization, "1"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization, "2"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization, "3"));
}

TEST_F(SensorDatabaseFixture, TestInsertIntrinsics) {
    EXPECT_THROW(database::InsertIntrinsics(db, sensor_name, CalibrationStep::PoseInitialization, CameraModel::Pinhole,
                                            {testing_utilities::pinhole_intrinsics}),
                 std::runtime_error);

    AddCamera();
    AddStep(CalibrationStep::PoseInitialization);

    EXPECT_NO_THROW(database::InsertIntrinsics(db, sensor_name, CalibrationStep::PoseInitialization,
                                               CameraModel::Pinhole, {testing_utilities::pinhole_intrinsics}));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbPoseData) {
    // Throws because the foreign key constraints are not met yet.
    EXPECT_THROW(AddPose(CalibrationStep::PoseInitialization), std::runtime_error);

    // Satisfy foreign key constraints.
    AddImage();
    AddTarget();
    AddStep(CalibrationStep::PoseInitialization);

    // Passes with no problem.
    EXPECT_NO_THROW(AddPose(CalibrationStep::PoseInitialization));
}

TEST_F(SensorDatabaseFixture, TestInsertReprojectionErrors) {
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};

    // Throws because the foreign key constraints are not met yet.
    EXPECT_THROW(database::InsertReprojectionErrors(db, sensor_name, CalibrationStep::PoseInitialization, data),
                 std::runtime_error);

    // Satisfy foreign key constraints.
    AddImage();
    AddTarget();
    AddStep(CalibrationStep::PoseInitialization);
    AddPose(CalibrationStep::PoseInitialization);

    EXPECT_NO_THROW(database::InsertReprojectionErrors(db, sensor_name, CalibrationStep::PoseInitialization, data));
}

TEST(DatabaseSensorDataInterface, TestInsertImuData) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    std::string_view sensor_name_1{"/imu/polaris/123"};
    EXPECT_NO_THROW(database::InsertImuData(db, sensor_name_1,
                                            ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                                            {1, {Vector3d::Zero(), Vector3d::Zero()}}}));

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary
    // key (timestamp_ns, sensor_name) so it is not a duplicate
    std::string_view sensor_name_2{"/imu/polaris/456"};
    EXPECT_NO_THROW(
        database::InsertImuData(db, sensor_name_2, ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}}}));

    // Add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW(database::InsertImuData(db, sensor_name_2, ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}}}),
                 std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestInsertImuErrors) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};
    std::string_view sensor_name{"/imu/polaris/123"};

    // Try to add a record before the foreign key requirements are met - not gonna work!
    EXPECT_THROW(database::InsertImuErrors(db, sensor_name, CalibrationStep::ExtrinsicInitialization,
                                           ImuErrors{{0, {Vector3d::Zero(), Vector3d::Zero()}}}),
                 std::runtime_error);

    // Satisfy foreign key requirements - an imu error requires a corresponding imu measurement and calibratio step.
    database::InsertImuData(db, sensor_name, ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}}});
    database::InsertStep(db, sensor_name, CalibrationStep::ExtrinsicInitialization, "");

    // Happy path.
    EXPECT_NO_THROW(database::InsertImuErrors(db, sensor_name, CalibrationStep::ExtrinsicInitialization,
                                              ImuErrors{{0, {Vector3d::Zero(), Vector3d::Zero()}}}));

    // Try to add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW(database::InsertImuErrors(db, sensor_name, CalibrationStep::ExtrinsicInitialization,
                                           ImuErrors{{0, {Vector3d::Zero(), Vector3d::Zero()}}}),
                 std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestInsertControlPoints) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    database::InsertStep(db, "/cam/retro/123", CalibrationStep::SplineInterpolation, "");
    database::InsertStep(db, "/cam/retro/456", CalibrationStep::SplineInterpolation, "");

    spline::Matrix2NXd const control_points{spline::Matrix2NXd::Random(6, 10)};

    std::string_view sensor_name_1{"/cam/retro/123"};
    EXPECT_NO_THROW(
        database::InsertControlPoints(db, sensor_name_1, CalibrationStep::SplineInterpolation, control_points));

    std::string_view sensor_name_2{"/cam/retro/456"};
    EXPECT_NO_THROW(
        database::InsertControlPoints(db, sensor_name_2, CalibrationStep::SplineInterpolation, control_points));

    EXPECT_THROW(database::InsertControlPoints(db, sensor_name_2, CalibrationStep::SplineInterpolation, control_points),
                 std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestInsertTimeHandler) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    database::InsertStep(db, "/cam/retro/123", CalibrationStep::SplineInterpolation, "");
    database::InsertStep(db, "/cam/retro/456", CalibrationStep::SplineInterpolation, "");

    spline::TimeHandler const time_handler{100, 200};

    std::string_view sensor_name_1{"/cam/retro/123"};
    EXPECT_NO_THROW(database::InsertTimeHandler(db, sensor_name_1, CalibrationStep::SplineInterpolation, time_handler));

    std::string_view sensor_name_2{"/cam/retro/456"};
    EXPECT_NO_THROW(database::InsertTimeHandler(db, sensor_name_2, CalibrationStep::SplineInterpolation, time_handler));

    EXPECT_THROW(database::InsertTimeHandler(db, sensor_name_2, CalibrationStep::SplineInterpolation, time_handler),
                 std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestInsertGravity) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    // WARN(Jack): Similar to the case for the extrinsic (see test below), we are hijacking the sensor_name here.
    // Gravity is not really directly associated with any single sensor. If anything it is more related to the target
    // because that is what sets the world coordinate frame.
    std::string_view sensor_name{"world"};
    database::InsertStep(db, sensor_name, CalibrationStep::ExtrinsicInitialization, "");

    Array3d const gravity_w{0, 1, 2};
    EXPECT_NO_THROW(database::InsertGravity(db, sensor_name, CalibrationStep::ExtrinsicInitialization, gravity_w));

    EXPECT_THROW(database::InsertGravity(db, sensor_name, CalibrationStep::ExtrinsicInitialization, gravity_w),
                 std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestInsertExtrinsic) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    // WARN(Jack): We are hacking the sensor_name of the extrinsic calibration table to actually be the name of the
    // transform. This is a hack! The extrinsic table is the first time that we came across datat that was related to
    // two sensors, and as of today (02.06.26) our database design does not handle this, and therefore we are restoring
    // to hacks :(
    std::string_view sensor_name{"tf_imu_co"};
    database::InsertStep(db, sensor_name, CalibrationStep::ExtrinsicInitialization, "");

    Array6d const tf_imu_co{0, 1, 2, 3, 4, 5};
    EXPECT_NO_THROW(database::InsertExtrinsic(db, sensor_name, CalibrationStep::ExtrinsicInitialization, tf_imu_co));

    EXPECT_THROW(database::InsertExtrinsic(db, sensor_name, CalibrationStep::ExtrinsicInitialization, tf_imu_co),
                 std::runtime_error);
}