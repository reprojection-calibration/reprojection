#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct SplineInitialization {
    // These members are only needed to calculate the reprojection error - they are not actually used for the spline
    // initialization/interpolation.
    CameraInfo camera_info;
    CameraMeasurements targets;
    OptimizationState bundle;

    // TODO(Jack): Should we rename the CalibrationStep to SplineInitialization?
    CalibrationStep step_type{CalibrationStep::SplineInterpolation};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    spline::Se3Spline Compute() const;

    spline::Se3Spline Load(SqlitePtr const db) const;

    void Save(spline::Se3Spline const& spline, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
