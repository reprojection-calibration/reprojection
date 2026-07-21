#include "steps/initialize_calibration.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/generated/minimum_config.hpp"

using namespace reprojection;

// TODO(Jack): Once we refactor the init logic to handle different recordings and config runs we need to test those
// logically too!

TEST(StepsInitializeCalibration, TestHappyPath) {
    auto db{database::CalibrationDatabase(":memory:", true)};
    toml::table const cfg_table{toml::parse(testing_utilities::minimum_config)};

    steps::CalibrationContext const result{steps::InitializeCalibration(cfg_table, db)};
    EXPECT_EQ(result.recording_id.value, 1);
    EXPECT_EQ(result.run_id.value, 1);
    EXPECT_EQ(result.camera_id.value, 1);
    EXPECT_EQ(result.target_id.value, 2);
    ASSERT_TRUE(result.imu_id.has_value());
    EXPECT_EQ(result.imu_id->value, 3);
}