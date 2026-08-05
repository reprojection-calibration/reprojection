#pragma once

#include <ranges>

#include "database/calibration_database.hpp"

using namespace reprojection;

class StepTestFixture : public ::testing::Test {
   protected:
    StepId InsertCameraInfo(CameraInfo const& camera_info) {
        auto const step_id{db_.GetOrCreateStep(StepType::CameraInfo, "").first};
        db_.CameraInfoInsert(step_id, camera_id_, camera_info);

        return step_id;
    }

    StepId InsertIntrinsics(CameraModel const model, CameraState const& intrinsics) {
        auto const step_id{db_.GetOrCreateStep(StepType::IntrinsicInit, "").first};
        db_.IntrinsicInsert(step_id, camera_id_, model, intrinsics);

        return step_id;
    }

    StepId InsertImages(EncodedImages const& images) {
        auto const step_id{db_.GetOrCreateStep(StepType::ImageLoading, "").first};
        db_.ImagesInsert(step_id, camera_id_, images);

        return step_id;
    }

    StepId InsertExtractedTargets(CameraMeasurements const& targets) {
        // Targets have a foreign key to images, so manufacture exactly the image rows required by the targets.
        EncodedImages images;
        for (auto const timestamp_ns : targets | std::views::keys) {
            images.emplace(timestamp_ns, ImageBuffer{});
        }
        auto const image_loading_id{InsertImages(images)};

        auto const target_step_id{db_.GetOrCreateStep(StepType::FeatureExtraction, "").first};
        db_.ExtractedTargetsInsert(target_step_id, image_loading_id, camera_id_, targets);

        return target_step_id;
    }

    database::CalibrationDatabase db_{":memory:", true};
    AssetId camera_id_{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
};
