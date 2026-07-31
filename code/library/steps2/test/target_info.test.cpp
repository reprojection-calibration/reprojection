#include "steps/target_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

using namespace reprojection;

TEST(StepsTargetInfo, TestImuTargetInfoStepRunner) {
    auto db{database::CalibrationDatabase(":memory:", true)};
    RecordingId const recording_id{db.GetOrCreateRecording("", "")};
    auto const owner{steps::StepOwner::Recording(recording_id)};

    AssetId const target_id{db.GetOrCreateAsset(AssetType::Target, 0, "")};
    config::Config::Target const target{TargetType::Aprilgrid3, {1, 2}};
    steps::TargetInfoStep const step{target_id, target};

    StepId const step_id{RunStep<steps::TargetInfoStep>(owner, step, db)};

    auto const result{db.TargetInfoSelect(step_id, target_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(result->height, target.size[0]);
    EXPECT_EQ(result->width, target.size[1]);
}

TEST(StepsTargetInfo, Xxx) { EXPECT_EQ(1, 2); }