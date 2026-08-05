#include "steps/target_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

using namespace reprojection;

TEST(StepsTargetInfo, TestTargetInfoStepRunner) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    AssetId const target_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "")};
    config::Config::Target const target{TargetType::Aprilgrid3, {1, 2}};
    steps::TargetInfoStep const step{target_id, target};

    StepId const step_id{RunStep<steps::TargetInfoStep>(step, db)};

    auto const result{database::TargetInfoSelect(db.get(), step_id, target_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(result->height, target.size[0]);
    EXPECT_EQ(result->width, target.size[1]);
}

TEST(StepsTargetInfo, TestTargetInfoStep) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    AssetId const target_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "")};
    config::Config::Target const target{TargetType::Aprilgrid3, {1, 2}};

    steps::TargetInfoStep const step{target_id, target};
    EXPECT_EQ(step.Type(), StepType::TargetInfo);
    EXPECT_EQ(step.CacheKey().value, "03d5d5226dde69073fd8d1b0813738058bfb75bff151dfced9b636374ae0ff5b");

    auto const [step_id, _]{database::GetOrCreateStep(db.get(), StepType::TargetInfo, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db));

    auto const result{database::TargetInfoSelect(db.get(), step_id, target_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(result->height, target.size[0]);
    EXPECT_EQ(result->width, target.size[1]);
}