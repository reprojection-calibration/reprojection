#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct FeatureExtractionStep {
    std::string sensor_name;
    std::shared_ptr<EncodedImages> images;
    TargetInfo target_info;
    bool show_extraction;

    CalibrationStep step_type{CalibrationStep::FeatureExtraction};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    CameraMeasurements Compute() const;

    CameraMeasurements Load(SqlitePtr const db) const;

    void Save(CameraMeasurements const& extracted_targets, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
