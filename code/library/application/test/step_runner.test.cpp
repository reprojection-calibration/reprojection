#include "application/step_runner.hpp"

#include <gtest/gtest.h>

#include "application/steps.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

class StepRunnerFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        auto const mvg_data{testing_mocks::GenerateMvgData(camera_info, camera_state, 50, 1e9)};
        targets = mvg_data.first;

        db = std::make_shared<database::CalibrationDatabase>(":memory:", true, false);
        database::WriteToDb(camera_info, db);
    }

    std::shared_ptr<database::CalibrationDatabase> db;
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState camera_state{testing_utilities::pinhole_intrinsics};
    CameraMeasurements targets;
};

// TODO(Jack): How can we write a test to test the cascading delete and step replacement logic?
TEST_F(StepRunnerFixture, TestStepRunnerWithLpiStep) {
    application::LinearPoseInitializationStep const step{camera_info, targets, camera_state};

    auto [frames, cache_status]{RunStep<Frames>(step, db)};
    EXPECT_EQ(cache_status, application::CacheStatus::CacheMiss);

    // On rerun with the same inputs it will be a cache hit
    std::tie(frames, cache_status) = RunStep<Frames>(step, db);
    EXPECT_EQ(cache_status, application::CacheStatus::CacheHit);
}