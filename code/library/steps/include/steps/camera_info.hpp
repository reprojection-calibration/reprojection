#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct CameraInfoStep {
    CameraInfoStep(AssetId camera_id, std::string_view serialized_image_sampler, ImageSampler const& image_sampler,
                   CameraModel camera_model);

    static StepType Type() { return StepType::CameraInfo; }

    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    Hash image_sampler_hash_;
    ImageSampler image_sampler_;
    CameraModel camera_model_;
};

}  // namespace reprojection::steps
