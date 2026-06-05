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

// TODO(Jack): Refactor the tests that do not currently have a test fixture to either use the existing test fixture or
// add and imu test fixture and a extrinsic calibration test fixture to find a place for them.

class CameraDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::InsertEntity(db, sensor_name, Entity::Camera);
    }

    void AddStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        database::InsertStep(db, sensor_name, step_name, cache_key);
    }

    void AddCameraInfo() const {
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

TEST_F(CameraDatabaseFixture, TestInsertCameraInfo) {
    EXPECT_NO_THROW(AddCameraInfo());
    EXPECT_THROW(AddCameraInfo(), std::runtime_error);  // Duplicate entry not allowed!
}

// TODO(Jack): If we have foreign key constraints one day, like we will have to for the multi-target case, then we can
// add helpers like we have in the test fixture for other types, but for now we do not need to add a target info to the
// database for any other reason.
TEST_F(CameraDatabaseFixture, TestInsertTargetInfo) {
    database::InsertStep(db, sensor_name, CalibrationStep::TargetInfo, "");

    EXPECT_NO_THROW(database::InsertTargetInfo(db, sensor_name, TargetInfo{TargetType::Aprilgrid3, 8, 6, 0.1, false}));
    EXPECT_THROW(database::InsertTargetInfo(db, sensor_name, TargetInfo{TargetType::Aprilgrid3, 8, 6, 0.1, false}),
                 std::runtime_error);
}

TEST_F(CameraDatabaseFixture, TestInsertImages) {
    EXPECT_NO_THROW(AddImage());
    EXPECT_THROW(AddImage(), std::runtime_error);
}

TEST_F(CameraDatabaseFixture, TestInsertTargets) {
    EXPECT_THROW(database::InsertTargets(db, sensor_name, CameraMeasurements{{timestamp_ns, {}}}), std::runtime_error);

    AddImage();
    database::InsertStep(db, sensor_name, CalibrationStep::FeatureExtraction, "");

    EXPECT_NO_THROW(database::InsertTargets(db, sensor_name, CameraMeasurements{{timestamp_ns, {}}}));
}

TEST_F(CameraDatabaseFixture, TestWriteToDbCalibrationStep) {
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization));
    EXPECT_NO_THROW(AddStep(CalibrationStep::BundleAdjustment));
    EXPECT_NO_THROW(AddStep(CalibrationStep::SplineInitialization));
}

TEST_F(CameraDatabaseFixture, TestWriteToDbCalibrationStepUpsert) {
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization, "1"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization, "2"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::PoseInitialization, "3"));
}

TEST_F(CameraDatabaseFixture, TestInsertIntrinsics) {
    EXPECT_THROW(database::InsertIntrinsics(db, sensor_name, CalibrationStep::PoseInitialization, CameraModel::Pinhole,
                                            {testing_utilities::pinhole_intrinsics}),
                 std::runtime_error);

    AddCameraInfo();
    AddStep(CalibrationStep::PoseInitialization);

    EXPECT_NO_THROW(database::InsertIntrinsics(db, sensor_name, CalibrationStep::PoseInitialization,
                                               CameraModel::Pinhole, {testing_utilities::pinhole_intrinsics}));
}

TEST_F(CameraDatabaseFixture, TestWriteToDbPoseData) {
    // Throws because the foreign key constraints are not met yet.
    EXPECT_THROW(AddPose(CalibrationStep::PoseInitialization), std::runtime_error);

    // Satisfy foreign key constraints.
    AddImage();
    AddTarget();
    AddStep(CalibrationStep::PoseInitialization);

    // Passes with no problem.
    EXPECT_NO_THROW(AddPose(CalibrationStep::PoseInitialization));
}

TEST_F(CameraDatabaseFixture, TestInsertReprojectionErrors) {
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

TEST(DatabaseSensorDataInterface, TestInsertControlPoints) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    database::InsertStep(db, "/cam/retro/123", CalibrationStep::SplineInitialization, "");
    database::InsertStep(db, "/cam/retro/456", CalibrationStep::SplineInitialization, "");

    spline::Matrix2NXd const control_points{spline::Matrix2NXd::Random(6, 10)};

    std::string_view sensor_name_1{"/cam/retro/123"};
    EXPECT_NO_THROW(
        database::InsertControlPoints(db, sensor_name_1, CalibrationStep::SplineInitialization, control_points));

    std::string_view sensor_name_2{"/cam/retro/456"};
    EXPECT_NO_THROW(
        database::InsertControlPoints(db, sensor_name_2, CalibrationStep::SplineInitialization, control_points));

    EXPECT_THROW(
        database::InsertControlPoints(db, sensor_name_2, CalibrationStep::SplineInitialization, control_points),
        std::runtime_error);
}

TEST(DatabaseSensorDataInterface, TestInsertTimeHandler) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    database::InsertStep(db, "/cam/retro/123", CalibrationStep::SplineInitialization, "");
    database::InsertStep(db, "/cam/retro/456", CalibrationStep::SplineInitialization, "");

    spline::TimeHandler const time_handler{100, 200};

    std::string_view sensor_name_1{"/cam/retro/123"};
    EXPECT_NO_THROW(
        database::InsertTimeHandler(db, sensor_name_1, CalibrationStep::SplineInitialization, time_handler));

    std::string_view sensor_name_2{"/cam/retro/456"};
    EXPECT_NO_THROW(
        database::InsertTimeHandler(db, sensor_name_2, CalibrationStep::SplineInitialization, time_handler));

    EXPECT_THROW(database::InsertTimeHandler(db, sensor_name_2, CalibrationStep::SplineInitialization, time_handler),
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