#include "steps/image_loading.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;



TEST_F(ImageLoadingFixture, TestImageLoadingStepRunner) {
    steps::ImageLoading const step{camera_id_, "", image_sampler_};
    StepId const step_id{RunStep<steps::ImageLoading>(context_.workflow_id, step, db_)};

    auto const result{database::ImagesSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), std::size(encoded_images_));

    for (auto const timestamp_ns : encoded_images_ | std::views::keys) {
        EXPECT_EQ(std::size(result.at(timestamp_ns).data), std::size(encoded_images_.at(timestamp_ns).data));
    }
}

TEST_F(ImageLoadingFixture, TestImageLoadingStep) {
    // Build the step and check that the type and hash function are correct.
    steps::ImageLoading const step{camera_id_, "", image_sampler_};
    EXPECT_EQ(step.Type(), StepType::ImageLoading);
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    // Load the images and compare them to the known input.
    auto const result{database::ImagesSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), std::size(encoded_images_));

    for (auto const timestamp_ns : encoded_images_ | std::views::keys) {
        EXPECT_EQ(std::size(result.at(timestamp_ns).data), std::size(encoded_images_.at(timestamp_ns).data));
    }
}