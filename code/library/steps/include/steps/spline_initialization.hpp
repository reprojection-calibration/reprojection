#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct SplineInitialization {
    SplineInitialization(AssetId camera_id, StepId camera_poses_id, StepId targets_id, StepId camera_info_id,
                         StepId intrinsics_id, SqlitePtr db);

    static StepType Type() { return StepType::SplineInit; }

    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    Frames camera_poses_;
    // NOTE(Jack): These are only needed for the reprojection error calculation. They are not needed for the spline
    // initialization at all. But we do the diagnostic calculations in the spline init/other steps directly to avoid
    // creating dedicated diagnostic calculation steps - even if it means passing some unexpected information in.
    StepId targets_id_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
};

}  // namespace reprojection::steps
