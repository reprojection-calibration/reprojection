#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::steps {

struct ExtrinsicOptimization {
    ExtrinsicOptimization(AssetId camera_id, AssetId imu_id, StepId targets_id, StepId imu_data_id, int num_threads,
                          StepId camera_info_id, StepId intrinsic_id, StepId spline_id, StepId extrinsic_id,
                          StepId gravity_id, database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::ExtrinsicOptimization; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_id_;
    AssetId imu_id_;
    CameraMeasurements targets_;
    ImuMeasurements imu_data_;
    int num_threads_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
    std::unique_ptr<spline::Se3Spline> spline_;
    // TODO(Jack): Does it make sense to combine extrinsic and gravity into one type?
    // TODO(Jack): Should we name this to reflect its the initial guess?
    database::Extrinsic2 extrinsic_;
    Vector3d gravity_;
};

}  // namespace reprojection::steps
