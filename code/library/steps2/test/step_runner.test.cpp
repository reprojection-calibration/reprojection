#include "steps/step_runner.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

struct ExampleStep {
    static StepType Type() { return StepType::ImageLoading; }

    static Hash CacheKey() { return ""; }

    static void Execute(StepId const step_id, database::CalibrationDatabase& db) {
        (void)db;
        (void)step_id;

        return;
    }
};

TEST(StepsStepRunner, TestExampleStep) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    ExampleStep step;
    StepId result{steps::RunStep<ExampleStep>(step, db)};
    EXPECT_EQ(result.value, 1);

    // Rerunning the step should be a cache hit (but we can't see that here) and should return the same step ID
    result = steps::RunStep<ExampleStep>(step, db);
    EXPECT_EQ(result.value, 1);

    // TODO(Jack): Is there a way to run the step again and get another step id out of it? Right now the cache key is
    // hardcoded and a static method which means we cannot easily change it to trigger a cache miss and new step
    // creation.
}