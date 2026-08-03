#include "steps/intrinsic_initialization.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

class IntrinsicInitializationFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        camera_info_id_ = db_.GetOrCreateStep(StepType::CameraInfo, "").first;
        CameraInfo const camera_info{CameraModel::DoubleSphere, testing_utilities::image_bounds};
        db_.CameraInfoInsert(camera_info_id_, camera_id_, camera_info);

        // NOTE(Jack): The setup required here is pretty wild. The reason we need to do this is that we need targets to
        // actually exercise the intrinsic init step. Getting these into the db means that we also need images in the db
        // to satisfy the extracted target foreign key constraint. That is what this big mess here is doing. If we use
        // this in multiple places put it in one fixture.
        auto const [targets, _]{
            testing_mocks::GenerateMvgData(camera_info, {testing_utilities::double_sphere_intrinsics}, 11, 1)};

        EncodedImages const images{[&targets]() {
            EncodedImages images;
            for (auto const timestamp_ns : targets | std::ranges::views::keys) {
                images.insert({timestamp_ns, {}});
            }
            return images;
        }()};

        StepId const image_loading_id{db_.GetOrCreateStep(StepType::ImageLoading, "").first};
        db_.ImagesInsert(image_loading_id, camera_id_, images);

        targets_id_ = db_.GetOrCreateStep(StepType::FeatureExtraction, "").first;
        db_.ExtractedTargetsInsert(targets_id_, image_loading_id, camera_id_, targets);
    }

    database::CalibrationDatabase db_{database::CalibrationDatabase(":memory:", true)};
    AssetId camera_id_{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
    StepId camera_info_id_{-1};
    StepId targets_id_{-1};
};

TEST_F(IntrinsicInitializationFixture, TestIntrinsicInitializationStepRunner) {
    steps::IntrinsicInitialization const step{camera_id_, 1, camera_info_id_, targets_id_, db_};
    StepId const step_id{RunStep<steps::IntrinsicInitialization>(step, db_)};

    auto const result{db_.IntrinsicSelect(step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    Array5d const gt_result{530.372, 360, 240, 0, 0.5};  // Heuristic!
    EXPECT_TRUE(result->intrinsics.isApprox(gt_result, 1e-3));
}

TEST_F(IntrinsicInitializationFixture, TestIntrinsicInitializationStep) {
    steps::IntrinsicInitialization const step{camera_id_, 1, camera_info_id_, targets_id_, db_};
    EXPECT_EQ(step.Type(), StepType::IntrinsicInitialization);
    EXPECT_EQ(step.CacheKey().value, "5f0399afd6e6b0ba1e282ed54d1dab16219d7a1eb4ecec30a237fd6eee95f348");

    auto const [step_id, _]{db_.GetOrCreateStep(StepType::IntrinsicInitialization, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{db_.IntrinsicSelect(step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    Array5d const gt_result{530.372, 360, 240, 0, 0.5};  // Heuristic!
    EXPECT_TRUE(result->intrinsics.isApprox(gt_result, 1e-3));
}