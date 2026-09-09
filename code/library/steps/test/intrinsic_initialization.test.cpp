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
        // WARN(Jack): The methods below can only be called after the base fixtures SetUp method has been called!
        StepTestFixture::SetUp();

        // NOTE(Jack): For the "single camera" workflow we arbitrarily select the first camera.
        camera_id_ = context_.assets.cameras.front().id;
        camera_info_id_ = InsertCameraInfo(camera_id_);
        targets_id_ = InsertExtractedTargets(camera_id_);
    }

    AssetId camera_id_;
    StepId camera_info_id_;
    StepId targets_id_;
};

TEST_F(IntrinsicInitializationFixture, TestIntrinsicInitializationStepRunner) {
    steps::IntrinsicInitialization const step{camera_id_, 1, camera_info_id_, targets_id_, db_};
    StepId const step_id{RunStep<steps::IntrinsicInitialization>(context_.workflow_id, step, db_)};

    auto const result{database::IntrinsicSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    Array5d const gt_result{530.372, 360, 240, 0, 0.5};  // Heuristic!
    EXPECT_TRUE(result->value.isApprox(gt_result, 1e-3));
}

TEST_F(IntrinsicInitializationFixture, TestIntrinsicInitializationStep) {
    steps::IntrinsicInitialization const step{camera_id_, 1, camera_info_id_, targets_id_, db_};
    EXPECT_EQ(step.Type(), StepType::IntrinsicInit);
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "4fca2c782d81fcebe010feed2bc34a6d3eb75f575956154f3c18ce70f0218058");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::IntrinsicInit, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::IntrinsicSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    Array5d const gt_result{530.372, 360, 240, 0, 0.5};  // Heuristic!
    EXPECT_TRUE(result->value.isApprox(gt_result, 1e-3));
}