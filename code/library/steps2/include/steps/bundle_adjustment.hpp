#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"

namespace reprojection::steps {

struct BundleAdjustment {
    BundleAdjustment(AssetId camera_id, StepId targets_id, int num_threads, StepId camera_info_id, StepId intrinsic_id,
                     StepId camera_poses_id, database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::BundleAdjustment; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase const& db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    int num_threads_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
    Frames camera_poses_;
};

Frames AlignRotations(Frames data);

}  // namespace reprojection::steps
