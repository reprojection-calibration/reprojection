#pragma once

#include "database/calibration_database.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// NOTE(Jack): I had originally planned to not store the images in the database because it would require more
// reading/writing than just feeding the images directly into the feature extractor. But it leads to the database
// getting large. The benefit is that it makes our downstream workflow and database visualization extremely consistent.
// TODO(Jack): We really need to find a way for the feature extractor to process the image stream itself and not also
// write the images to the db!

struct ImageLoading {
    ImageLoading(AssetId camera_id, std::string_view serialized_image_sampler, ImageSampler const& image_sampler);

    static StepType Type() { return StepType::ImageLoading; }

    Hash CacheKey(database::CalibrationDatabase& db) const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_id_;
    Hash cache_key_;
    ImageSampler image_sampler_;
};

}  // namespace reprojection::steps
