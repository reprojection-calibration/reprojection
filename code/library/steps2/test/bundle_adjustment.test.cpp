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
        CameraInfo const camera_info{CameraModel::DoubleSphere, testing_utilities::image_bounds};
        camera_info_id_ = InsertCameraInfo(camera_info);

        auto const [targets, poses]{
            testing_mocks::GenerateMvgData(camera_info, {testing_utilities::double_sphere_intrinsics}, 11, 1)};
        targets_id_ = InsertExtractedTargets(targets);

        intrinsics_id_ =
            InsertIntrinsics(CameraModel::DoubleSphere, CameraState{testing_utilities::double_sphere_intrinsics});

        // TODO(Jack): Do these poses need to be inverted? Not that correctness really matters here but we should keep
        // it in mind that something might be flipped around here.
        db_.CameraPosesInsert(pose_init_id_, targets_id_, camera_id_, poses);
    }

    StepId camera_info_id_;
    StepId targets_id_;
    StepId intrinsics_id_;
    StepId pose_init_id_{db_.GetOrCreateStep(StepType::PoseInitialization, "").first};
};

TEST_F(BundleAdjustmentFixture, TestBundleAdjustmentStepRunner) {
    steps::BundleAdjustment const step{camera_id_, targets_id_, 1, camera_info_id_, intrinsics_id_, pose_init_id_, db_};
    StepId const step_id{RunStep<steps::BundleAdjustment>(step, db_)};

    auto const result{db_.CameraPosesSelect(step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);

    auto const result2{db_.IntrinsicSelect(step_id, camera_id_)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_TRUE(result2->intrinsics.isApprox(testing_utilities::double_sphere_intrinsics));
}

TEST_F(BundleAdjustmentFixture, TestBundleAdjustmentStep) {
    steps::BundleAdjustment const step{camera_id_, targets_id_, 1, camera_info_id_, intrinsics_id_, pose_init_id_, db_};
    EXPECT_EQ(step.Type(), StepType::BundleAdjustment);
    EXPECT_EQ(step.CacheKey().value, "0dae470cd3c711a1692153ee4ccf969c5e4ccb5da30dbb0df40e2fdac600dc8e");

    auto const [step_id, _]{db_.GetOrCreateStep(StepType::BundleAdjustment, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{db_.CameraPosesSelect(step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 7);

    auto const result2{db_.IntrinsicSelect(step_id, camera_id_)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_TRUE(result2->intrinsics.isApprox(testing_utilities::double_sphere_intrinsics));
}

TEST(StepsBundleAdjustment, TestAlignRotations) {
    Vector6d const pose_1{1, 0, 0, 0, 0, 0};
    Vector6d const pose_2{-1 * pose_1};
    OptimizationState const input{{}, {Frame{1, pose_1}, Frame{2, pose_2}}};

    OptimizationState const output{steps::AlignRotations(input)};

    EXPECT_TRUE(output.frames.at(1).pose.isApprox(output.frames.at(2).pose));
}