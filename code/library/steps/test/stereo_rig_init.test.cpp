#include "steps/stereo_rig_init.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class StereoRigInitFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        cam0_ = context_.assets.cameras.front().id;

        for (auto const& [id, _] : context_.assets.cameras) {
            StepId const targets_id{InsertExtractedTargets(id)};
            StepId const poses_id{InsertPoses(id, targets_id)};

            cam_stages_.push_back({id, {0}, {0}, {0}, poses_id});
        }

        // Just make sure this isn't accidentally empty and we iterate over nothing.
        ASSERT_EQ(std::size(cam_stages_), 2);
    }

    AssetId cam0_;
    std::vector<CamStageIds> cam_stages_;
};

TEST_F(StereoRigInitFixture, TestStereoRigInitStepRunner) {
    steps::StereoRigInit const step{cam0_, 0, cam_stages_, db_};
    StepId const step_id{RunStep<steps::StereoRigInit>(context_.workflow_id, step, db_)};

    for (auto const& cam : cam_stages_) {
        auto const result{database::ExtrinsicSelect(db_.get(), step_id, cam.asset_id, cam0_)};
        ASSERT_TRUE(result.has_value());

        EXPECT_NEAR(result->se3_a_b.sum(), 0, 1e-12);  // Identity!
        EXPECT_EQ(result->frame_a, cam.asset_id);
        EXPECT_EQ(result->frame_b, cam0_);
    }
}

TEST_F(StereoRigInitFixture, TestStereoRigInitStep) {
    steps::StereoRigInit const step{cam0_, 0, cam_stages_, db_};

    EXPECT_EQ(step.Type(), StepType::StereoRigInit);
    std::vector const gt_assets{cam0_, context_.assets.cameras.back().id};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "cdb3147fbcc9ca2299fc3c10ace0f92af1331886a862f5e9804f79df89e77d9b");

    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::StereoRigInit, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    for (auto const& cam : cam_stages_) {
        auto const result{database::ExtrinsicSelect(db_.get(), step_id, cam.asset_id, cam0_)};
        ASSERT_TRUE(result.has_value());

        EXPECT_NEAR(result->se3_a_b.sum(), 0, 1e-12);
        EXPECT_EQ(result->frame_a, cam.asset_id);
        EXPECT_EQ(result->frame_b, cam0_);
    }
}
