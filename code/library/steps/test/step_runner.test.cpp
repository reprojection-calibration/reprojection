#include "steps/step_runner.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

struct ExampleStep {
    static StepType Type() { return StepType::FeatureExtraction; }

    std::vector<AssetId> Assets() const { return {asset_id_}; }

    Hash CacheKey() const { return cache_key_; }

    static void Execute(StepId const step_id, SqlitePtr const db) {
        (void)db;
        (void)step_id;

        return;
    }

    // We need to be able to associate this step with its' asset group.
    AssetId asset_id_;
    // We only have this here for the testing purpose below so we can change it manually and trigger a cache miss!
    Hash cache_key_{""};
};

TEST(StepsStepRunner, TestExampleStep) {
    auto db{database::OpenCalibDb(":memory:", true)};

    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};
    database::AssetGroupInsert(db.get(), {asset_id});
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), WorkflowType::CamImu, {asset_id})};

    ExampleStep step{asset_id};
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

struct ExampleStepThrow {
    static StepType Type() { return StepType::FeatureExtraction; }

    std::vector<AssetId> Assets() const { return {asset_id_}; }

    Hash CacheKey() const { return ""; }

    static void Execute(StepId const step_id, SqlitePtr const db) {
        (void)db;
        (void)step_id;

        throw std::runtime_error("");

        return;
    }

    // We need to be able to associate this step with its' asset group.
    AssetId asset_id_;
};

TEST(StepsStepRunner, TestExecutionFailCleanUp) {
    auto db{database::OpenCalibDb(":memory:", true)};

    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};
    database::AssetGroupInsert(db.get(), {asset_id});
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db.get(), WorkflowType::CamImu, {asset_id})};

    ExampleStepThrow step{asset_id};
    EXPECT_THROW(steps::RunStep<ExampleStepThrow>(workflow_id, step, db), std::runtime_error);

    // By inserting a step here and checking that it's the first possible ID and a cache miss we are implicitly checking
    // that the failed step cleanup logic in steps::RunStep worked properly and did not leave any step artifacts in the
    // database.
    auto step_insert{database::GetOrCreateStep(db.get(), StepType::FeatureExtraction, "")};
    EXPECT_EQ(step_insert.first.value, 1);
    EXPECT_EQ(step_insert.second, CacheStatus::CacheMiss);
}