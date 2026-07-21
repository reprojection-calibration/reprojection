#pragma once

#include "database/calibration_database.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// NOTE(Jack): I had originally planned to not store the images in the database because it would require more
// reading/writing than just feeding the images directly into the feature extractor. But it leads to the database
// getting large. The benefit is that it makes our downstream workflow and database visualization extremely consistent.

struct ImageLoading {
    AssetId camera_id_;
    Hash cache_key_;
    ImageSampler image_sampler_;

    ImageLoading(AssetId const camera_id, std::string_view serialized_image_sampler, ImageSampler const& image_sampler);

    static StepType Type() { return StepType::ImageLoading; }

    Hash CacheKey(database::CalibrationDatabase& db);

    void Execute(database::CalibrationDatabase& db, StepId const step_id);
};

}  // namespace reprojection::steps
