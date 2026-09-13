#include "steps/pose_init.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class PoseInitFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        camera_id_ = context_.assets.cameras.front().id;
        camera_info_id_ = InsertCameraInfo(camera_id_);
        targets_id_ = InsertExtractedTargets(camera_id_);
        intrinsics_id_ = InsertIntrinsic(camera_id_);
    }

    AssetId camera_id_;
    StepId camera_info_id_;
    StepId targets_id_;
    StepId intrinsics_id_;
};

TEST_F(PoseInitFixture, TestPoseInitStepRunner) {
    steps::PoseInit const step{camera_id_, targets_id_, camera_info_id_, intrinsics_id_, db_};
    StepId const step_id{RunStep<steps::PoseInit>(context_.workflow_id, step, db_)};

    auto const result{database::CameraPosesSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);
}

TEST_F(PoseInitFixture, TestPoseInitStep) {
    steps::PoseInit const step{camera_id_, targets_id_, camera_info_id_, intrinsics_id_, db_};
    EXPECT_EQ(step.Type(), StepType::PoseInit);
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "723245d956786cad6abadb69629b5bccc8db6596c0864a6c77380c9f818351a1");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::PoseInit, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::CameraPosesSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);
}