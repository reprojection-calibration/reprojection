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

using namespace reprojection;
using PoseType = database::PoseType;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(DatabaseSensorDataInterface, TestAddExtractedTargetData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t const timestamp_ns{0};
    std::string const sensor_name{"/cam/retro/123"};

    // Adding a target with no corresponding image database entry is invalid! Foreign key constraint :)
    EXPECT_THROW(AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db), std::runtime_error);

    database::AddImage(timestamp_ns, sensor_name, db);
    EXPECT_NO_THROW(AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddPoseData) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t const timestamp_ns{0};
    Frames const data{{timestamp_ns, {Array6d::Zero()}}};
    std::string const sensor_name{"/cam/retro/123"};

    // Throws because it fails the "CHECK()" constraint in the calibration_steps table.
    EXPECT_THROW(database::AddPoseData(data, "invalid_step", sensor_name, db), std::runtime_error);

    // TODO(Jack): We need to create an enum for these step names! Or something to make this less error prone.
    EXPECT_NO_THROW(database::AddPoseData(data, "linear_pose_initialization", sensor_name, db));
    EXPECT_NO_THROW(database::AddPoseData(data, "nonlinear_refinement", sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddReprojectionError) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    uint64_t const timestamp_ns{0};
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};
    std::string const sensor_name{"/cam/retro/123"};

    // Fails foreign key constraint because there is no corresponding camera_poses table entry yet
    EXPECT_THROW(database::AddReprojectionError(data, "linear_pose_initialization", sensor_name, db),
                 std::runtime_error);
    EXPECT_THROW(database::AddReprojectionError(data, "nonlinear_refinement", sensor_name, db), std::runtime_error);

    database::AddImage(timestamp_ns, sensor_name, db);
    AddExtractedTargetData({timestamp_ns, {}}, sensor_name, db);

    Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
    database::AddPoseData(frames, "linear_pose_initialization", sensor_name, db);
    database::AddPoseData(frames, "nonlinear_refinement", sensor_name, db);

    EXPECT_NO_THROW(database::AddReprojectionError(data, "linear_pose_initialization", sensor_name, db));
    EXPECT_NO_THROW(database::AddReprojectionError(data, "nonlinear_refinement", sensor_name, db));
}

TEST(DatabaseSensorDataInterface, TestAddImuData) {
    TemporaryFile const temp_file{".db3"};
    auto const db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true)};

    std::string_view sensor_name_1{"/imu/polaris/123"};
    EXPECT_NO_THROW(database::AddImuData({{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                          {1, {Vector3d::Zero(), Vector3d::Zero()}}},
                                         sensor_name_1, db));

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary
    // key (timestamp_ns, sensor_name) so it is not a duplicate
    std::string_view sensor_name_2{"/imu/polaris/456"};
    EXPECT_NO_THROW(database::AddImuData({{0, {Vector3d::Zero(), Vector3d::Zero()}}}, sensor_name_2, db));

    // Add a repeated record - this is not successful because the primary key must always be unique!
    EXPECT_THROW(database::AddImuData({{0, {Vector3d::Zero(), Vector3d::Zero()}}}, sensor_name_2, db),
                 std::runtime_error);
}
