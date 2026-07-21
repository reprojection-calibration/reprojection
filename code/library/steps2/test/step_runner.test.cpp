#include "steps/step_runner.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

struct ExampleStep {
    static StepType Type() { return StepType::ImageLoading; }

    static Hash CacheKey(database::CalibrationDatabase& db) {
        (void)db;

        return "";
    }

    static void Execute(database::CalibrationDatabase& db, StepId const step_id) {
        (void)db;
        (void)step_id;

        return;
    }
};

// TODO(Jack): How can we write a test to test the cascading delete and step replacement logic?
TEST(StepsStepRunner, TestExampleStep) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    RecordingId const recording_id{db.GetOrCreateRecording("recording.bag", "sha256-xxx")};
    auto owner{steps::StepOwner::Recording(recording_id)};

    ExampleStep step;
    StepId result{RunStep<ExampleStep>(owner, step, db)};
    EXPECT_EQ(result.value, 1);

    // Rerunning the step should be a cache hit (but we can't see that here) and should return the same step ID
    result = RunStep<ExampleStep>(owner, step, db);
    EXPECT_EQ(result.value, 1);

    // Now add another instance of the step owned by a run. Normally we probably would not have one step once owned by a
    // recording and again by a run, but we do it here so we can see the step id increment.
    RunId const run_id{db.GetOrCreateRun(recording_id, "")};
    owner = steps::StepOwner::Run(run_id);

    result = RunStep<ExampleStep>(owner, step, db);
    EXPECT_EQ(result.value, 2);
}