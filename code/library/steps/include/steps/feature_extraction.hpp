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
    // TODO(Jack): Long term we should parse the app config at the top level and pass it we do camera_info/target_info.
    // But for now we will just put it here.
    toml::table app_config;

    CalibrationStep step_type{CalibrationStep::FeatureExtraction};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    CameraMeasurements Compute() const;

    CameraMeasurements Load(SqlitePtr const db) const;

    void Save(CameraMeasurements const& extracted_targets, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
