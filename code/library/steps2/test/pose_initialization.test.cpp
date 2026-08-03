#include "steps/pose_initialization.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class PoseInitializationFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        CameraInfo const camera_info{CameraModel::DoubleSphere, testing_utilities::image_bounds};
        camera_info_id_ = InsertCameraInfo(camera_info);

        auto const [targets, _]{
            testing_mocks::GenerateMvgData(camera_info, {testing_utilities::double_sphere_intrinsics}, 11, 1)};
        targets_id_ = InsertExtractedTargets(targets);

        intrinsics_id_ =
            InsertIntrinsics(CameraModel::DoubleSphere, CameraState{testing_utilities::double_sphere_intrinsics});
    }

    StepId camera_info_id_;
    StepId targets_id_;
    StepId intrinsics_id_;
};

TEST_F(PoseInitializationFixture, TestPoseInitializationStepRunner) {
    steps::PoseInitialization const step{camera_id_, targets_id_, camera_info_id_, intrinsics_id_, db_};
    StepId const step_id{RunStep<steps::PoseInitialization>(step, db_)};

    auto const result{db_.CameraPosesSelect(step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);
}

TEST_F(PoseInitializationFixture, TestPoseInitializationStep) {
    steps::PoseInitialization const step{camera_id_, targets_id_, camera_info_id_, intrinsics_id_, db_};
    EXPECT_EQ(step.Type(), StepType::PoseInitialization);
    EXPECT_EQ(step.CacheKey().value, "723245d956786cad6abadb69629b5bccc8db6596c0864a6c77380c9f818351a1");

    auto const [step_id, _]{db_.GetOrCreateStep(StepType::PoseInitialization, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{db_.CameraPosesSelect(step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);
}