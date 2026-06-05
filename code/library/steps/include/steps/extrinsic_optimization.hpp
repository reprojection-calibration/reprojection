#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct ExtrinsicOptimization {
    ImuMeasurements imu_data;
    spline::Se3Spline spline;
    Array6d tf_imu_co;
    Array3d gravity_w;
    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState intrinsics;

    CalibrationStep step_type{CalibrationStep::ExtrinsicOptimization};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    // TODO(Jack): Use a real defined type?
    std::tuple<spline::Se3Spline, Array6d, Array3d> Compute() const;

    std::tuple<spline::Se3Spline, Array6d, Array3d> Load(SqlitePtr const db) const;

    void Save(std::tuple<spline::Se3Spline, Array6d, Array3d> const& state, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
