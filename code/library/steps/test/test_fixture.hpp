#pragma once

#include <ranges>

#include <testing_utilities/generated/calibration_config.hpp>

#include "database/calibration_database.hpp"
#include "steps/initialize_workflow.hpp"

using namespace reprojection;

class StepTestFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        // NOTE(Jack): Here we manually edit the second camera to be a pinhole camera so that way we do not have two
        // cameras with the exact same data! That could interfere with cache hits if the data is the exact same across
        // the two sensors.
        toml::table config{toml::parse(testing_utilities::calibration_config)};
        config["cam1"].as_table()->insert_or_assign("camera_model", "pinhole");

        context_ = steps::InitializeCalibration(config, db_);
    }

    StepId InsertCameraInfo(AssetId const camera_id) {
        auto const camera{GetCamera(camera_id)};
        CameraInfo const camera_info{camera.config.camera_model, testing_utilities::image_bounds};

        StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::CameraInfo, "").first};
        database::CameraInfoInsert(db_.get(), step_id, camera.id, camera_info);

        return step_id;
    }

    StepId InsertImages(AssetId const camera_id, ImageSamples const& images) {
        auto const step_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
        database::ImagesInsert(db_.get(), step_id, camera_id, images);

        return step_id;
    }

    StepId InsertExtractedTargets(AssetId const camera_id) {
        auto const camera{GetCamera(camera_id)};
        CameraInfo const camera_info{camera.config.camera_model, testing_utilities::image_bounds};

        Intrinsic intrinsic;
        if (camera_info.camera_model == CameraModel::DoubleSphere) {
            intrinsic = {testing_utilities::double_sphere_intrinsics};
        } else if (camera_info.camera_model == CameraModel::Pinhole) {
            intrinsic = {testing_utilities::pinhole_intrinsics};
        } else {
            throw std::runtime_error{std::format("Camera model {} not found!", ToString(camera_info.camera_model))};
        }

        // ERROR(Jack): Use common timing parameterization across all methods!
        auto const [targets, _]{testing_mocks::GenerateMvgData(camera_info, intrinsic, 11, 1)};

        // Initialize empty image data using the target timestamps and then write them to the db to satisfy the foreign
        // key constraint.
        ImageSamples const images{[&targets] {
            ImageSamples images;
            for (auto const timestamp_ns : targets | std::views::keys) {
                images.emplace(timestamp_ns, ImageBuffer{});
            }
            return images;
        }()};
        StepId const image_loading_id{InsertImages(camera.id, images)};

        StepId const target_step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};
        database::TargetsInsert(db_.get(), target_step_id, image_loading_id, camera.id, targets);

        return target_step_id;
    }

   private:
    Asset<config::Config::Camera> GetCamera(AssetId const camera_id) {
        auto const& cameras{context_.assets.cameras};
        auto const it{std::ranges::find_if(context_.assets.cameras,
                                           [camera_id](auto const& camera) { return camera.id == camera_id; })};
        if (it == std::cend(cameras)) {
            throw std::runtime_error{std::format("Camera asset id {} not found!", camera_id.value)};
        }

        return *it;
    }

    /**
    StepId InsertIntrinsics(CameraModel const model, Intrinsic const& intrinsics) {
        auto const step_id{database::GetOrCreateStep(db_.get(), StepType::IntrinsicInit, "").first};
        database::IntrinsicInsert(db_.get(), step_id, camera_id_, model, intrinsics);

        return step_id;
    }

    StepId InsertImages(ImageSamples const& images) {
        auto const step_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
        database::ImagesInsert(db_.get(), step_id, camera_id_, images);

        return step_id;
    }

    StepId InsertExtractedTargets(TargetSamples const& targets) {
        // Targets have a foreign key to images, so manufacture exactly the image rows required by the targets.
        ImageSamples images;
        for (auto const timestamp_ns : targets | std::views::keys) {
            images.emplace(timestamp_ns, ImageBuffer{});
        }
        auto const image_loading_id{InsertImages(images)};

        auto const target_step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};
        database::TargetsInsert(db_.get(), target_step_id, image_loading_id, camera_id_, targets);

        return target_step_id;
    }

    **/

   public:
    SqlitePtr db_{database::OpenCalibrationDatabase(":memory:", true)};
    steps::CalibrationContext context_;
};
