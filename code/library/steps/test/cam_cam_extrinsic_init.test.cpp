#include "steps/cam_cam_extrinsic_init.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class CamCamExtrinsicInitFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        // NOTE(Jack): The order here is the opposite of whats expected! We kinda invert the order, meaning 'a' is the
        // second camera and 'b' the first camera. We do this because the order of the transformation in the extrinsic
        // initialization is from the first camera to all other cameras and we want to maintain some sort of consistency
        // with our se3_a_b extrinsic notation even if this might be a little misleading.
        camera_a_id_ = context_.assets.cameras.back().id;
        camera_b_id_ = context_.assets.cameras.front().id;

        for (auto const& [id, _] : context_.assets.cameras) {
            StepId const targets_id{InsertExtractedTargets(id)};
            StepId const poses_id{InsertPoses(id, targets_id)};

            calibrations_.push_back({id, {0}, {0}, {0}, poses_id});
        }
    }

    AssetId camera_a_id_;
    AssetId camera_b_id_;

    std::vector<CamStageIds> calibrations_;
};

TEST_F(CamCamExtrinsicInitFixture, TestExtrinsicInitStepRunner) {
    steps::StereoRigInit const step{calibrations_, 0, db_};
    StepId const step_id{RunStep<steps::StereoRigInit>(context_.workflow_id, step, db_)};

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, camera_a_id_, camera_b_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);  // Heuristic!
    EXPECT_EQ(result->frame_a, camera_a_id_);
    EXPECT_EQ(result->frame_b, camera_b_id_);
}

TEST_F(CamCamExtrinsicInitFixture, TestExtrinsicInitStep) {
    steps::StereoRigInit const step{calibrations_, 0, db_};

    EXPECT_EQ(step.Type(), StepType::ExtrinsicInit);
    std::vector const gt_assets{camera_b_id_, camera_a_id_};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "cdb3147fbcc9ca2299fc3c10ace0f92af1331886a862f5e9804f79df89e77d9b");

    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ExtrinsicInit, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, camera_a_id_, camera_b_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);
    EXPECT_EQ(result->frame_a, camera_a_id_);
    EXPECT_EQ(result->frame_b, camera_b_id_);
}
