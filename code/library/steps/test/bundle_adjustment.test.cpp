#include "steps/bundle_adjustment.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class BundleAdjustmentFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        camera_id_ = context_.assets.cameras.front().id;
        camera_info_id_ = InsertCameraInfo(camera_id_);
        targets_id_ = InsertExtractedTargets(camera_id_);
        intrinsics_id_ = InsertIntrinsic(camera_id_);
        pose_init_id_ = InsertPoses(camera_id_, targets_id_);
    }

    AssetId camera_id_;
    StepId camera_info_id_;
    StepId targets_id_;
    StepId intrinsics_id_;
    StepId pose_init_id_;
};

TEST_F(BundleAdjustmentFixture, TestBundleAdjustmentStepRunner) {
    steps::BundleAdjustment const step{camera_id_, targets_id_, 1, camera_info_id_, intrinsics_id_, pose_init_id_, db_};
    StepId const step_id{RunStep<steps::BundleAdjustment>(context_.workflow_id, step, db_)};

    auto const result{database::CameraPosesSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);

    auto const result2{database::IntrinsicSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_TRUE(result2->value.isApprox(testing_utilities::double_sphere_intrinsics));
}

TEST_F(BundleAdjustmentFixture, TestBundleAdjustmentStep) {
    steps::BundleAdjustment const step{camera_id_, targets_id_, 1, camera_info_id_, intrinsics_id_, pose_init_id_, db_};
    EXPECT_EQ(step.Type(), StepType::BundleAdjustment);
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "0dae470cd3c711a1692153ee4ccf969c5e4ccb5da30dbb0df40e2fdac600dc8e");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::BundleAdjustment, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::CameraPosesSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);

    auto const result2{database::IntrinsicSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_TRUE(result2->value.isApprox(testing_utilities::double_sphere_intrinsics));
}
