#pragma once


#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

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

}  // namespace reprojection::application
