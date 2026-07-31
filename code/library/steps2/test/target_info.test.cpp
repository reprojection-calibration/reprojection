#include "steps/target_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

using namespace reprojection;

TEST(StepsTargetInfo, TestTargetInfoStepRunner) {
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

TEST(StepsTargetInfo, TestTargetInfoStep) {
    auto db{database::CalibrationDatabase(":memory:", true)};
    AssetId const target_id{db.GetOrCreateAsset(AssetType::Target, 0, "")};
    config::Config::Target const target{TargetType::Aprilgrid3, {1, 2}};

    steps::TargetInfoStep const step{target_id, target};
    EXPECT_EQ(step.Type(), StepType::TargetInfo);
    EXPECT_EQ(step.CacheKey(db).value, "03d5d5226dde69073fd8d1b0813738058bfb75bff151dfced9b636374ae0ff5b");

    RecordingId const recording_id{db.GetOrCreateRecording("", "")};
    auto const [step_id, _]{db.GetOrCreateStep(recording_id, std::nullopt, StepType::TargetInfo, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db));

    auto const result{db.TargetInfoSelect(step_id, target_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(result->height, target.size[0]);
    EXPECT_EQ(result->width, target.size[1]);
}