#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct ImageLoadingStep {
    std::string sensor_name;
    std::string cache_key;
    ImageSource image_source;

    CalibrationStep step_type{CalibrationStep::ImageLoading};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    EncodedImages Compute() const;

    EncodedImages Load(SqlitePtr const db) const;

    void Save(EncodedImages const& images, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
