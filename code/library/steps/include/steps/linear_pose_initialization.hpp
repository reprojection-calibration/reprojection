#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct LpiStep {
    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState camera_state;

    CalibrationStep step_type{CalibrationStep::LinearPoseInitialization};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    Frames Compute() const;

    Frames Load(SqlitePtr const db) const;

    void Save(Frames const& frames, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
