#include "steps/cam_cam_extrinsic_optimization.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class CamCamExtrinsicOptimizationFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        camera_a_id_ = context_.assets.cameras.back().id;
        camera_b_id_ = context_.assets.cameras.front().id;

        for (auto const& [id, _] : context_.assets.cameras) {
            StepId const camera_info_id{InsertCameraInfo(id)};
            StepId const targets_id{InsertExtractedTargets(id)};
            StepId const bundle_adjustment_id{InsertIntrinsic(id)};

            // We only need to insert the poses for the reference camera (i.e. the rig poses)
            if (id == camera_b_id_) {
                // NOTE(Jack): For the reference camera we need to simulate a bundle adjustment step so we need to write
                // the cache key so that the InsertPoses() call below can add to the cached step.
                database::StepCacheKeyUpdate(db_.get(), bundle_adjustment_id, "");

                StepId const result{InsertPoses(id, targets_id)};
                if (result != bundle_adjustment_id) {
                    throw std::runtime_error(
                        "The intrinsic and pose id must be the same to simulate the bundle adjustment step!");
                }
            }

            calibrations_.push_back({id, camera_info_id, targets_id, {0}, bundle_adjustment_id});
        }

        extrinsic_id_ = InsertExtrinsic(camera_a_id_, camera_b_id_);
    }

    AssetId camera_a_id_;
    AssetId camera_b_id_;

    std::vector<CamStageIds> calibrations_;
    StepId extrinsic_id_;
};

TEST_F(CamCamExtrinsicOptimizationFixture, TestExtrinsicInitStepRunner) {
    steps::CamCamExtrinsicOptimization const step{calibrations_, extrinsic_id_, 1, 0, db_};
    StepId const step_id{RunStep<steps::CamCamExtrinsicOptimization>(context_.workflow_id, step, db_)};

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, camera_a_id_, camera_b_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);  // Heuristic!
    EXPECT_EQ(result->frame_a, camera_a_id_);
    EXPECT_EQ(result->frame_b, camera_b_id_);
}

TEST_F(CamCamExtrinsicOptimizationFixture, TestExtrinsicInitStep) {
    steps::CamCamExtrinsicOptimization const step{calibrations_, extrinsic_id_, 1, 0, db_};

    EXPECT_EQ(step.Type(), StepType::ExtrinsicOptimization);
    std::vector const gt_assets{camera_b_id_, camera_a_id_};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "e2dce0022f9fc213b45a9b5cbeccf9f63892de59b0f224ce02c8c40fa7670830");

    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ExtrinsicInit, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, camera_a_id_, camera_b_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);
    EXPECT_EQ(result->frame_a, camera_a_id_);
    EXPECT_EQ(result->frame_b, camera_b_id_);
}
