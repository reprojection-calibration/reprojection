#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"

namespace reprojection::steps {

struct CameraInfoStep {
    CameraInfoStep(AssetId camera_id, StepId image_loading_id, CameraModel camera_model,
                   database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::CameraInfo; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase const& db) const;

   private:
    AssetId camera_id_;
    CameraModel camera_model_;
    // TODO(Jack): It is way overkill to load all the images here just to get the size of the first one. We need a
    // better iamge handling pipeline across the image loading/feature extraction/camera info. We need an entirely new
    // concept.
    std::shared_ptr<EncodedImages> images_;
};

}  // namespace reprojection::steps
