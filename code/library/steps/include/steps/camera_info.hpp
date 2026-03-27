#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

struct CameraInfoStep {
    // TODO(Jack): Should we make a central definition of this?
    using ImageProvider = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

    std::string cache_key;
    // TODO(Jack): We should have structs here not unparsed toml tables, right?
    toml::table sensor_config;
    ImageProvider image_source;

    CalibrationStep step_type{CalibrationStep::CameraInfo};

    std::string SensorName() const { return sensor_config["camera_name"].as_string()->get(); }

    std::string CacheKey() const;

    CameraInfo Compute() const;

    CameraInfo Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(CameraInfo const& camera_info, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

}  // namespace reprojection::application
