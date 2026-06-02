#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct SplineInterpolation {
    std::string sensor_name;
    Frames poses;

    CalibrationStep step_type{CalibrationStep::SplineInterpolation};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    spline::Se3Spline Compute() const;

    spline::Se3Spline Load(SqlitePtr const db) const;

    void Save(spline::Se3Spline const& spline, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
