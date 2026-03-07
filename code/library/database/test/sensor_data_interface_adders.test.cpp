#include "database/sensor_data_interface_adders.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "testing_utilities/constants.hpp"
#include "testing_utilities/temporary_file.hpp"
#include "types/sensor_data_types.hpp"

#include "sqlite3_helpers.hpp"

// TODO(Jack): We need to create an enum for these step names! Or something to make this less error prone.

using namespace reprojection;

class SensorDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override { db = std::make_shared<database::CalibrationDatabase>(":memory:", true, false); }

    void AddImage() const { database::AddImage(timestamp_ns, sensor_name, db); }

    void AddTarget() const { WriteToDb(sensor_name, CameraMeasurement{timestamp_ns, {}}, db); }

    void AddStep(std::string const& step) const { WriteToDb(step, db); }

    void AddPose(std::string const& step) const {
        Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
        WriteToDb(step, sensor_name, frames, db);
    }

    std::shared_ptr<database::CalibrationDatabase> db;
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/cam/retro/123"};
};

TEST_F(SensorDatabaseFixture, AddExtractedTargetData) {
    EXPECT_THROW(WriteToDb(sensor_name, CameraMeasurement{timestamp_ns, {}}, db), std::runtime_error);

    AddImage();

    EXPECT_NO_THROW(WriteToDb(sensor_name, CameraMeasurement{timestamp_ns, {}}, db));
}

TEST_F(SensorDatabaseFixture, AddCalibrationStep) {
    EXPECT_THROW(database::WriteToDb("invalid_step", db), std::runtime_error);

    EXPECT_NO_THROW(AddStep("linear_pose_initialization"));
    EXPECT_NO_THROW(AddStep("nonlinear_refinement"));
}

TEST_F(SensorDatabaseFixture, TestAddPoseData) {
    // Throws because the calibration step linear_pose_initialization has not been added to the database yet.
    EXPECT_THROW(AddPose("linear_pose_initialization"), std::runtime_error);

    AddStep("linear_pose_initialization");

    EXPECT_NO_THROW(AddPose("linear_pose_initialization"));
}

TEST_F(SensorDatabaseFixture, TestAddReprojectionError) {
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};

    // Fails foreign key constraint because there is no corresponding poses table entry yet
    EXPECT_THROW(database::WriteToDb("linear_pose_initialization", sensor_name, data, db), std::runtime_error);

    AddImage();
    AddTarget();
    WriteToDb("linear_pose_initialization", db);
    AddPose("linear_pose_initialization");

    EXPECT_NO_THROW(database::WriteToDb("linear_pose_initialization", sensor_name, data, db));
}

TEST(DatabaseSensorDataInterface, TestAddImuData) {
    testing_utilities::TemporaryFile const temp_file{".db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true)};

    std::string_view sensor_name_1{"/imu/polaris/123"};
    EXPECT_NO_THROW(database::WriteToDb(sensor_name_1,
                                        {{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                         {1, {Vector3d::Zero(), Vector3d::Zero()}}},
                                        db));

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary
    // key (timestamp_ns, sensor_name) so it is not a duplicate
    std::string_view sensor_name_2{"/imu/polaris/456"};
    EXPECT_NO_THROW(database::WriteToDb(sensor_name_2, {{0, {Vector3d::Zero(), Vector3d::Zero()}}}, db));

    // Add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW(database::WriteToDb(sensor_name_2, {{0, {Vector3d::Zero(), Vector3d::Zero()}}}, db),
                 std::runtime_error);
}
