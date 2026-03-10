#include "database/database_write.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

#include "sqlite3_helpers.hpp"

using namespace reprojection;

class SensorDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override { db = std::make_shared<database::CalibrationDatabase>(":memory:", true, false); }

    void AddCamera() const {
        database::WriteToDb(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddImage() const { database::AddImage(timestamp_ns, sensor_name, db); }

    void AddTarget() const { WriteToDb(CameraMeasurement{timestamp_ns, {}}, sensor_name, db); }

    void AddStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        WriteToDb(step_name, cache_key, sensor_name, db);
    }

    void AddPose(CalibrationStep const step_name) const {
        Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
        WriteToDb(frames, step_name, sensor_name, db);
    }

    std::shared_ptr<database::CalibrationDatabase> db;
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/cam/retro/123"};
};

TEST_F(SensorDatabaseFixture, WriteCameraInfo) {
    EXPECT_NO_THROW(AddCamera());
    EXPECT_THROW(AddCamera(), std::runtime_error);  // Duplicate entry not allowed!
}

TEST_F(SensorDatabaseFixture, AddExtractedTargetData) {
    EXPECT_THROW(WriteToDb(CameraMeasurement{timestamp_ns, {}}, sensor_name, db), std::runtime_error);

    AddCamera();
    AddImage();

    EXPECT_NO_THROW(WriteToDb(CameraMeasurement{timestamp_ns, {}}, sensor_name, db));
}

TEST_F(SensorDatabaseFixture, AddCalibrationStep) {
    AddCamera();

    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Cnlr));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Sint));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Snlr));
}

TEST_F(SensorDatabaseFixture, AddCalibrationStepUpsert) {
    AddCamera();

    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi, "1"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi, "2"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi, "3"));
}

TEST_F(SensorDatabaseFixture, TestAddCameraIntrinsic) {
    EXPECT_THROW(database::WriteToDb({testing_utilities::pinhole_intrinsics}, CameraModel::Pinhole,
                                     CalibrationStep::Lpi, sensor_name, db),
                 std::runtime_error);

    AddCamera();
    AddStep(CalibrationStep::Lpi);

    EXPECT_NO_THROW(database::WriteToDb({testing_utilities::pinhole_intrinsics}, CameraModel::Pinhole,
                                        CalibrationStep::Lpi, sensor_name, db));
}

TEST_F(SensorDatabaseFixture, TestAddPoseData) {
    // Throws because the calibration step linear_pose_initialization has not been added to the database yet.
    EXPECT_THROW(AddPose(CalibrationStep::Lpi), std::runtime_error);

    AddCamera();
    AddStep(CalibrationStep::Lpi);

    EXPECT_NO_THROW(AddPose(CalibrationStep::Lpi));
}

TEST_F(SensorDatabaseFixture, TestAddReprojectionError) {
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};

    // Fails foreign key constraint because there is no corresponding poses table entry yet
    EXPECT_THROW(database::WriteToDb(data, CalibrationStep::Lpi, sensor_name, db), std::runtime_error);

    AddCamera();
    AddImage();
    AddTarget();
    AddStep(CalibrationStep::Lpi);
    AddPose(CalibrationStep::Lpi);

    EXPECT_NO_THROW(database::WriteToDb(data, CalibrationStep::Lpi, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddImuData) {
    auto const db{std::make_shared<database::CalibrationDatabase>(":memory:", true)};

    std::string_view sensor_name_1{"/imu/polaris/123"};
    EXPECT_NO_THROW(database::WriteToDb({{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                         {1, {Vector3d::Zero(), Vector3d::Zero()}}},
                                        sensor_name_1, db));

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary
    // key (timestamp_ns, sensor_name) so it is not a duplicate
    std::string_view sensor_name_2{"/imu/polaris/456"};
    EXPECT_NO_THROW(database::WriteToDb({{0, {Vector3d::Zero(), Vector3d::Zero()}}}, sensor_name_2, db));

    // Add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW(database::WriteToDb({{0, {Vector3d::Zero(), Vector3d::Zero()}}}, sensor_name_2, db),
                 std::runtime_error);
}
