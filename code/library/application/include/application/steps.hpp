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

    ImageProvider image_source;
    std::string cache_key;
    // TODO THESE SHOULD ALREADY BE PARSED AND NOT STORED AS TABLES!
    toml::table target_config;
    toml::table sensor_config;

    CalibrationStep step_type{CalibrationStep::FtEx};

    std::string SensorName() const { return sensor_config["sensor_name"].as_string()->get(); }

    std::string CacheKey() const;

    CameraMeasurements Compute() const;

    CameraMeasurements Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(CameraMeasurements const& extracted_targets,
              std::shared_ptr<database::CalibrationDatabase> const db) const;
};

struct IntrinsicInitializationStep {
    CameraInfo camera_info;
    CameraMeasurements targets;

    CalibrationStep step_type{CalibrationStep::Ii};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    CameraState Compute() const;

    CameraState Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(CameraState const& frames, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

// TODO(Jack): Make private package source files one day when the application is whole.
struct LpiStep {
    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState camera_state;

    CalibrationStep step_type{CalibrationStep::Lpi};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    Frames Compute() const;

    Frames Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(Frames const& frames, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

struct CnlrStep {
    CameraInfo camera_info;
    CameraMeasurements targets;
    OptimizationState initial_state;

    CalibrationStep step_type{CalibrationStep::Cnlr};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    OptimizationState Compute() const;

    OptimizationState Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(OptimizationState const& optimized_state, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

}  // namespace reprojection::application
