#pragma once

#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct BundleAdjustment {
    BundleAdjustment(AssetId camera_id, StepId targets_id, int num_threads, StepId camera_info_id, StepId intrinsics_id,
                     StepId camera_poses_id, database::CalibrationDatabase& db);

    static StepType Type() { return StepType::BundleAdjustment; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    int num_threads_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
    Frames camera_poses_;
};

OptimizationState AlignRotations(OptimizationState state);

}  // namespace reprojection::steps
