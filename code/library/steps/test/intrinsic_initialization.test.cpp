#include "steps/intrinsic_initialization.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class IntrinsicInitializationFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        CameraInfo const camera_info{CameraModel::DoubleSphere, testing_utilities::image_bounds};
        camera_info_id_ = InsertCameraInfo(camera_info);

        auto const [targets, _]{
            testing_mocks::GenerateMvgData(camera_info, {testing_utilities::double_sphere_intrinsics}, 11, 1)};
        targets_id_ = InsertExtractedTargets(targets);
    }

    StepId camera_info_id_;
    StepId targets_id_;
};

TEST_F(IntrinsicInitializationFixture, TestIntrinsicInitializationStepRunner) {
    steps::IntrinsicInitialization const step{camera_id_, 1, camera_info_id_, targets_id_, db_};
    StepId const step_id{RunStep<steps::IntrinsicInitialization>(workflow_id_, step, db_)};

    auto const result{database::IntrinsicSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    Array5d const gt_result{530.372, 360, 240, 0, 0.5};  // Heuristic!
    EXPECT_TRUE(result->intrinsics.isApprox(gt_result, 1e-3));
}

TEST_F(IntrinsicInitializationFixture, TestIntrinsicInitializationStep) {
    steps::IntrinsicInitialization const step{camera_id_, 1, camera_info_id_, targets_id_, db_};
    EXPECT_EQ(step.Type(), StepType::IntrinsicInit);
    EXPECT_EQ(step.CacheKey().value, "5f0399afd6e6b0ba1e282ed54d1dab16219d7a1eb4ecec30a237fd6eee95f348");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::IntrinsicInit, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::IntrinsicSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    Array5d const gt_result{530.372, 360, 240, 0, 0.5};  // Heuristic!
    EXPECT_TRUE(result->intrinsics.isApprox(gt_result, 1e-3));
}