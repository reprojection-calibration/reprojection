#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// NOTE(Jack): I had originally planned to note store the images in the database because it would require more
// reading/writing than just feeding the images directly into the feature extractor. However, given that the images will
// be cached after the first execution there is really no argument not to do this. But it will lead to the database
// getting large (even though we only store .png encoded blobs). The benefit is that it makes our downstream workflow
// and database visualization extremely consistent.

struct ImageLoadingStep {
    std::string sensor_name;
    std::string cache_key;
    ImageSource image_source;

    CalibrationStep step_type{CalibrationStep::ImageLoading};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    std::shared_ptr<EncodedImages> Compute() const;

    std::shared_ptr<EncodedImages> Load(SqlitePtr const db) const;

    void Save(std::shared_ptr<EncodedImages const> const images, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
