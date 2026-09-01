#include "../include/steps/initialize_workflow.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"

using namespace reprojection;

TEST(ApplicationInitializeWorkflow, TestHappyPath) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};
    toml::table const cfg_table{toml::parse(testing_utilities::calibration_config)};

    // TODO(Jack): We could also test the specific assets ids but that is not really an invariant we need/want to
    // enforce, that is in an implementaiton detail.
    steps::CalibrationContext const result{steps::InitializeCalibration(cfg_table, db)};
    EXPECT_EQ(std::size(result.assets.All()), 4);
    EXPECT_TRUE(result.assets.imu.has_value());
    EXPECT_EQ(result.workflow_id.value, 1);
    EXPECT_EQ(result.workflow_type, WorkflowType::CamImu);
}

TEST(ApplicationInitializeWorkflow, TestDetermineWorkflowType) {
    toml::table const table{toml::parse(testing_utilities::calibration_config)};
    config::Config parsed_cfg{config::Config::Parse(table)};

    WorkflowType result{steps::DetermineWorkflowType(parsed_cfg)};
    EXPECT_EQ(result, WorkflowType::CamImu);

    // Drop the IMU so now its just a multi-cam workflow.
    parsed_cfg.imu = std::nullopt;
    result = steps::DetermineWorkflowType(parsed_cfg);
    EXPECT_EQ(result, WorkflowType::MultiCam);

    // Drop second camera so we get a single cam workflow.
    parsed_cfg.cameras.pop_back();
    result = steps::DetermineWorkflowType(parsed_cfg);
    EXPECT_EQ(result, WorkflowType::Cam);
}

TEST(ApplicationInitializeWorkflow, TestCreateCalibrationAssets) {
    toml::table const table{toml::parse(testing_utilities::calibration_config)};
    config::Config const parsed_cfg{config::Config::Parse(table)};
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    steps::CalibrationAssets const assets{steps::CreateCalibrationAssets(parsed_cfg, db)};

    // NOTE(Jack): The specific asset ids are not really an inherent property so much as a heuristic. If at some late
    // date the insertion logic changes then the id values might change and we will need to update them here. The real
    // actual invariants of this test is that there are two cameras and the imu is not std::nullopt.
    EXPECT_EQ(std::size(assets.cameras), 2);
    EXPECT_EQ(assets.cameras[0].id.value, 1);
    EXPECT_EQ(assets.cameras[1].id.value, 2);
    EXPECT_EQ(assets.target.id.value, 3);
    ASSERT_TRUE(assets.imu.has_value());
    EXPECT_EQ(assets.imu->id.value, 4);
}