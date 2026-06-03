#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

// TODO(Jack): Is it ok to consider gravity as part of the extrinsic calibration?

struct ExtrinsicInitialization {
    std::string sensor_name;
    ImuMeasurements imu_data;
    spline::Se3Spline spline;

    CalibrationStep step_type{CalibrationStep::ExtrinsicInitialization};

    std::string SensorName() const { return sensor_name; }

    std::string CacheKey() const;

    // TODO(Jack): Define types for extrinsics and gravity?
    std::pair<Array6d, Array3d> Compute() const;

    std::pair<Array6d, Array3d> Load(SqlitePtr const db) const;

    void Save(std::pair<Array6d, Array3d> const& extrinsic, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
