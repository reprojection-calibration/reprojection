#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct ExtrinsicOptimization {
    // These members are only needed to calculate the reprojection error - they are not actually used for the spline
    // initialization/interpolation.
    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState intrinsics;

    ImuMeasurements imu_data;
    spline::Se3Spline spline;
    ImuCamExtrinsic extrinsic;

    CalibrationStep step_type{CalibrationStep::ExtrinsicOptimization};

    std::string EntityId() const { return extrinsic.tf.EntityId(); }

    std::string HashInputs() const;

    // TODO(Jack): Do we really need to return the spline here? Do we need to return anything if this is gonna be the
    // final step? Theoretically speaking...
    std::pair<spline::Se3Spline, ImuCamExtrinsic> Compute() const;

    std::pair<spline::Se3Spline, ImuCamExtrinsic> Load(SqlitePtr const db) const;

    void Save(std::pair<spline::Se3Spline, ImuCamExtrinsic> const& data, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
