#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct ExtrinsicOptimization {
    CameraInfo camera_info_;
    CameraMeasurements targets_;
    CameraState intrinsics_;

    ImuMeasurements imu_data_;
    spline::Se3Spline spline_;
    ImuCamExtrinsic extrinsic_;

    static CalibrationStep StepType() { return CalibrationStep::ExtrinsicOptimization; }

    std::string EntityId() const { return extrinsic_.tf.EntityId(); }

    std::string HashInputs() const;

    std::pair<spline::Se3Spline, ImuCamExtrinsic> Compute() const;

    std::pair<spline::Se3Spline, ImuCamExtrinsic> Load(SqlitePtr const db) const;

    void Save(std::pair<spline::Se3Spline, ImuCamExtrinsic> const& data, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
