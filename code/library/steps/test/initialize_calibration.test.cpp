#include "steps/initialize_calibration.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"

using namespace reprojection;

// TODO(Jack): Once we refactor the init logic to handle different recordings and config runs we need to test those
// logically too!

TEST(StepsInitializeCalibration, TestHappyPath) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};
    toml::table const cfg_table{toml::parse(testing_utilities::calibration_config)};

    steps::CalibrationContext const result{steps::InitializeCalibration(cfg_table, db)};
    EXPECT_EQ(result.workflow_id.value, 1);
    EXPECT_EQ(result.camera_id.value, 1);
    EXPECT_EQ(result.target_id.value, 2);
    ASSERT_TRUE(result.imu_id.has_value());
    EXPECT_EQ(result.imu_id->value, 3);
}

TEST(StepsInitializeCalibration, TestDetermineWorkflowType) {
    toml::table const table{toml::parse(testing_utilities::calibration_config)};
    config::Config parsed_cfg{config::Config::Parse(table)};

    // Multicam-imu not yet supported.
    EXPECT_THROW(steps::DetermineWorkflowType(parsed_cfg), std::runtime_error);

    // Drop the IMU so now its just a multi-cam workflow.
    parsed_cfg.imu = std::nullopt;
    WorkflowType result{steps::DetermineWorkflowType(parsed_cfg)};
    EXPECT_EQ(result, WorkflowType::MultiCam);

    // Drop second camera so we get a single cam workflow.
    parsed_cfg.cameras.pop_back();
    result = steps::DetermineWorkflowType(parsed_cfg);
    EXPECT_EQ(result, WorkflowType::Cam);

    // Add the IMU back so we get a cam-imu workflow
    parsed_cfg.imu = config::Config::Imu::Parse(*table.get("imu")->as_table());
    result = steps::DetermineWorkflowType(parsed_cfg);
    EXPECT_EQ(result, WorkflowType::CamImu);

    // Add two cameras back for a total of three cameras which is not a supported multi-cam workflow yet.
    parsed_cfg.cameras.push_back(config::Config::Camera::Parse(*table.get("cam0")->as_table(), 10));
    parsed_cfg.cameras.push_back(config::Config::Camera::Parse(*table.get("cam1")->as_table(), 11));
    EXPECT_THROW(steps::DetermineWorkflowType(parsed_cfg), std::runtime_error);
}