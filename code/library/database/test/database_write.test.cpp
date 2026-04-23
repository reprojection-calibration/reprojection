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

    void AddCamera() const {
        database::WriteToDb(CalibrationStep::CameraInfo, "", sensor_name, db);
        database::WriteToDb(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddImage() const {
        database::WriteToDb(CalibrationStep::ImageLoading, "", sensor_name, db);
        database::WriteToDb(EncodedImages{{timestamp_ns, {}}}, sensor_name, db);
    }

    void AddStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        database::WriteToDb(step_name, cache_key, sensor_name, db);
    }

    void AddPose(CalibrationStep const step_name) const {
        Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
        database::WriteToDb(frames, step_name, sensor_name, db);
    }

    SqlitePtr db{nullptr};
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/cam/retro/123"};
};

TEST_F(SensorDatabaseFixture, TestWriteToDbCameraInfo) {
    EXPECT_NO_THROW(AddCamera());
    EXPECT_THROW(AddCamera(), std::runtime_error);  // Duplicate entry not allowed!
}

TEST_F(SensorDatabaseFixture, TestWriteToDbEncodedImages) {
    EXPECT_NO_THROW(AddImage());
    EXPECT_THROW(AddImage(), std::runtime_error);  // Duplicate entry not allowed!
}

TEST_F(SensorDatabaseFixture, TestWriteToDbCameraMeasurements) {
    EXPECT_THROW(database::WriteToDb(CameraMeasurements{{timestamp_ns, {}}}, sensor_name, db), std::runtime_error);

    AddImage();
    database::WriteToDb(CalibrationStep::FeatureExtraction, "", sensor_name, db);

    EXPECT_NO_THROW(database::WriteToDb(CameraMeasurements{{timestamp_ns, {}}}, sensor_name, db));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbCalibrationStep) {
    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi));
    EXPECT_NO_THROW(AddStep(CalibrationStep::CameraNonlinearRefinement));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Sint));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Snlr));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbCalibrationStepUpsert) {
    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi, "1"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi, "2"));
    EXPECT_NO_THROW(AddStep(CalibrationStep::Lpi, "3"));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbCameraIntrinsic) {
    EXPECT_THROW(database::WriteToDb({testing_utilities::pinhole_intrinsics}, CameraModel::Pinhole,
                                     CalibrationStep::Lpi, sensor_name, db),
                 std::runtime_error);

    AddCamera();
    AddStep(CalibrationStep::Lpi);

    EXPECT_NO_THROW(database::WriteToDb({testing_utilities::pinhole_intrinsics}, CameraModel::Pinhole,
                                        CalibrationStep::Lpi, sensor_name, db));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbPoseData) {
    // Throws because the calibration step linear_pose_initialization has not been added to the database yet.
    EXPECT_THROW(AddPose(CalibrationStep::Lpi), std::runtime_error);

    AddStep(CalibrationStep::Lpi);

    EXPECT_NO_THROW(AddPose(CalibrationStep::Lpi));
}

TEST_F(SensorDatabaseFixture, TestWriteToDbReprojectionError) {
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};

    // Fails foreign key constraint because there is no corresponding poses table entry yet
    EXPECT_THROW(database::WriteToDb(data, CalibrationStep::Lpi, sensor_name, db), std::runtime_error);

    AddStep(CalibrationStep::Lpi);
    AddPose(CalibrationStep::Lpi);

    EXPECT_NO_THROW(database::WriteToDb(data, CalibrationStep::Lpi, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestWriteToDbImuData) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    std::string_view sensor_name_1{"/imu/polaris/123"};
    EXPECT_NO_THROW(database::WriteToDb(ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                                        {1, {Vector3d::Zero(), Vector3d::Zero()}}},
                                        sensor_name_1, db));

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary
    // key (timestamp_ns, sensor_name) so it is not a duplicate
    std::string_view sensor_name_2{"/imu/polaris/456"};
    EXPECT_NO_THROW(database::WriteToDb(ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}}}, sensor_name_2, db));

    // Add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW(database::WriteToDb(ImuMeasurements{{0, {Vector3d::Zero(), Vector3d::Zero()}}}, sensor_name_2, db),
                 std::runtime_error);
}
