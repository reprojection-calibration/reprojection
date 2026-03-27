#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::application {

struct FeatureExtractionStep {
    std::string sensor_name;
    std::string cache_key;
    ImageSource image_source;
    // TODO(Jack): We should have structs here not unparsed toml tables, right?
    toml::table target_config;

    CalibrationStep step_type{CalibrationStep::FtEx};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    CameraMeasurements Compute() const;

    CameraMeasurements Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(CameraMeasurements const& extracted_targets,
              std::shared_ptr<database::CalibrationDatabase> const db) const;
};

}  // namespace reprojection::application
