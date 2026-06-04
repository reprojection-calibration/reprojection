#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

// ADD NOTE

// ADD NOTE

// ADD NOTE

// ADD NOTE
struct DoNotUse {};

struct SplineReprojectionError {
    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState intrinsics;
    spline::Se3Spline spline;

    CalibrationStep step_type;

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    DoNotUse Compute() const;

    DoNotUse Load(SqlitePtr const db) const;

    void Save(DoNotUse const do_not_use, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
