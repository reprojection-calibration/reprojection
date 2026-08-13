#include "steps/step_runner.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

struct ExampleStep {
    static StepType Type() { return StepType::ImageLoading; }

    Hash CacheKey() const { return cache_key_; }

    static void Execute(StepId const step_id, SqlitePtr const db) {
        (void)db;
        (void)step_id;

        return;
    }

    // We only have this here for the testing purpose below so we can change it manually and trigger a cache miss!
    Hash cache_key_{""};
};

TEST(StepsStepRunner, TestExampleStep) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), WorkflowType::CamImu, {asset_id})};

    ExampleStep step;
    StepId result{steps::RunStep<ExampleStep>(workflow_id, step, db)};
    EXPECT_EQ(result.value, 1);

    // Rerunning the step should be a cache hit (but we can't see that here) and should return the same step ID
    result = steps::RunStep<ExampleStep>(workflow_id, step, db);
    EXPECT_EQ(result.value, 1);

    // Change the cache key so we get a cache miss and a new step is created.
    step.cache_key_ = Hash{"1"};
    result = steps::RunStep<ExampleStep>(workflow_id, step, db);
    EXPECT_EQ(result.value, 2);
}