#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

// NOTE(Jack): The image input is unique because every data source is so different (ex. ROS1 bag vs. ROS2 bag) that the
// caller is responsible for calculating a cache key which characterizes the image data. Sure we could serialize all the
// image pixels inside of the step here, but that seems like overkill :) If one day we find out that we really can
// calculate a unique signature here easily then we can remove that code from the applications. My assumption is that
// serializing the image data is too much data because a single can be 20mb.
struct FeatureExtractionStep {
    // TODO(Jack): Should we make a central definition of this?
    using ImageProvider = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

    std::string sensor_name;
    std::string cache_key;
    ImageProvider image_source;
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
