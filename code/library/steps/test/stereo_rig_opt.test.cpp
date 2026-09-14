#include "steps/stereo_rig_opt.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class StereoRigOptFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        cam0_ = context_.assets.cameras.front().id;

        for (auto const& [id, _] : context_.assets.cameras) {
            StepId const camera_info_id{InsertCameraInfo(id)};
            StepId const targets_id{InsertExtractedTargets(id)};
            StepId const bundle_adjustment_id{InsertIntrinsic(id)};

            // We only need to insert the poses for the reference camera (i.e. the rig poses)
            if (id == cam0_) {
                // NOTE(Jack): For the reference camera we need to simulate a bundle adjustment step so we need to write
                // the cache key so that the InsertPoses() call below can add to the cached step.
                database::StepCacheKeyUpdate(db_.get(), bundle_adjustment_id, "");

                StepId const result{InsertPoses(id, targets_id)};
                if (result != bundle_adjustment_id) {
                    throw std::runtime_error(
                        "The intrinsic and pose id must be the same to simulate the bundle adjustment step!");
                }
            }

            cam_stages_.push_back({id, camera_info_id, targets_id, {0}, bundle_adjustment_id});

            // NOTE(Jack): We only really need to call StepCacheKeyUpdate() once to ensure both extrinsics get the same
            // step id but there is no downside to calling it twice that would justify increasing the code verbosity.
            extrinsic_id_ = InsertExtrinsic(id, cam0_);
            database::StepCacheKeyUpdate(db_.get(), extrinsic_id_, "");
        }

        // Just make sure this isn't accidentally empty and we iterate over nothing.
        ASSERT_EQ(std::size(cam_stages_), 2);
    }

    AssetId cam0_;
    std::vector<CamStageIds> cam_stages_;
    StepId extrinsic_id_;
};

TEST_F(StereoRigOptFixture, TestStereoRigOptRunner) {
    steps::StereoRigOpt const step{cam0_, cam_stages_, extrinsic_id_, 1, 0, db_};
    StepId const step_id{RunStep<steps::StereoRigOpt>(context_.workflow_id, step, db_)};

    for (auto const& cam : cam_stages_) {
        auto const result{database::ExtrinsicSelect(db_.get(), step_id, cam.asset_id, cam0_)};
        ASSERT_TRUE(result.has_value());

        EXPECT_NEAR(result->se3_a_b.sum(), 0, 1e-12);  // Identity!
        EXPECT_EQ(result->frame_a, cam.asset_id);
        EXPECT_EQ(result->frame_b, cam0_);
    }
}

TEST_F(StereoRigOptFixture, TestStereoRigOptStep) {
    steps::StereoRigOpt const step{cam0_, cam_stages_, extrinsic_id_, 1, 0, db_};

    EXPECT_EQ(step.Type(), StepType::StereoRigOpt);
    std::vector const gt_assets{cam0_, context_.assets.cameras.back().id};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "e2dce0022f9fc213b45a9b5cbeccf9f63892de59b0f224ce02c8c40fa7670830");

    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::StereoRigOpt, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    for (auto const& cam : cam_stages_) {
        auto const result{database::ExtrinsicSelect(db_.get(), step_id, cam.asset_id, cam0_)};
        ASSERT_TRUE(result.has_value());

        EXPECT_NEAR(result->se3_a_b.sum(), 0, 1e-12);
        EXPECT_EQ(result->frame_a, cam.asset_id);
        EXPECT_EQ(result->frame_b, cam0_);
    }
}
