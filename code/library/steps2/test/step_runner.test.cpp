#include "steps/step_runner.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

struct DummyStep {
    int result;

    static CalibrationStep StepType() { return CalibrationStep::PoseInitialization; }

    std::string EntityId() const { return ""; }

    std::string HashInputs() const { return std::to_string(result); };

    int Compute() const { return result; }

    int Load(SqlitePtr const db) const {
        (void)db;

        // NOTE(Jack): We add 100 here so we can differentiate in the test if the returned result comes from the
        // Compute() or Load() method.
        return result + 100;
    }

    void Save(int const data, SqlitePtr const db) const {
        (void)data;
        (void)db;
    }
};

// TODO(Jack): How can we write a test to test the cascading delete and step replacement logic?
TEST(StepsStepRunner, TestStepRunnerWithDummyStep) {
    auto db{database::OpenCalibrationDatabase(":memory:", true, false)};

    DummyStep step{2};

    // TODO(Jack): We need to find a clean way to incorporate the entity into the step testing. For now we add it here
    // manually and for all other setps.
    database::InsertEntity(db, step.EntityId(), Entity::Camera);

    auto [data, cache_status]{steps::RunStep<int>(step, db)};
    EXPECT_EQ(data, 2);  // Result from DummyStep.Compute()
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // On rerun with the same inputs it will be a cache hit
    std::tie(data, cache_status) = steps::RunStep<int>(step, db);
    EXPECT_EQ(data, 102);  // Result from DummyStep.Load()
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);

    // Change the value to see that we get a cache miss again
    step.result = 5;
    std::tie(data, cache_status) = steps::RunStep<int>(step, db);
    EXPECT_EQ(data, 5);  // Result from DummyStep.Compute()
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(data, cache_status) = steps::RunStep<int>(step, db);
    EXPECT_EQ(data, 105);  // Result from DummyStep.Load()
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}