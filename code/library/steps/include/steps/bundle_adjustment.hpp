#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct BundleAdjustment {
    BundleAdjustment(AssetId camera_id, StepId targets_id, int num_threads, StepId camera_info_id, StepId intrinsic_id,
                     StepId camera_poses_id, SqlitePtr db);

    static StepType Type() { return StepType::BundleAdjustment; }

    // TODO(Jack): Because we load the targets here it seems like we should also have the target assert ID here, but as
    // we do not strictly need it to load the targets, we do not force ourselves to use it. Are we missing the proper
    // abstraction?
    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    int num_threads_;
    TargetSamples targets_;
    CameraInfo camera_info_;
    Intrinsic intrinsics_;
    Frames camera_poses_;
};

}  // namespace reprojection::steps
