#include "steps/step_runner.hpp"

#include <gtest/gtest.h>

#include "steps/camera_nonlinear_refinement.hpp"

using namespace reprojection;

struct DummyStep {
    CalibrationStep step_type{CalibrationStep::Lpi};

    std::string SensorName() const { return ""; }

    std::string CacheKey() const { return ""; };

    int Compute() const { return 1; }

    int Load(SqlitePtr const db) const {
        (void)db;

        return 2;
    }

    void Save(int const data, SqlitePtr const db) const {
        (void)data;
        (void)db;
    }
};

// TODO(Jack): How can we write a test to test the cascading delete and step replacement logic?
TEST(stepsStepRunner, TestStepRunnerWithDummyStep) {
    auto db{database::OpenCalibrationDatabase(":memory:", true, false)};
    database::WriteToDb(CameraInfo{"", CameraModel::Pinhole, {}}, db);

    DummyStep const step;

    auto [data, cache_status]{steps::RunStep<int>(step, db)};
    EXPECT_EQ(data, 1);  // Result from DummyStep.Compute()
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // On rerun with the same inputs it will be a cache hit
    std::tie(data, cache_status) = steps::RunStep<int>(step, db);
    EXPECT_EQ(data, 2);  // Result from DummyStep.Load()
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}