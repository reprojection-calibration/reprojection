#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct IntrinsicInitializationStep {
    CameraInfo camera_info;
    CameraMeasurements targets;

    CalibrationStep step_type{CalibrationStep::Ii};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    CameraState Compute() const;

    CameraState Load(SqlitePtr const db) const;

    void Save(CameraState const& frames, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
