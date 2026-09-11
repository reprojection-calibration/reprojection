#pragma once

#include "optimization/extrinsic_optimization.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

// TODO(Jack): One day can we just have a single bundle adjustment step that generically does a "rig" whether it is
// single or multi-cam, I think we are not too far away.

namespace reprojection::steps {

struct StereoRigOpt {
    StereoRigOpt(std::vector<CamStageIds> const& cams, StepId extrinsic_init_id, int num_threads,
                 uint64_t approx_sync_delta_ns, SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicOptimization; }

    std::vector<AssetId> Assets() const {
        std::vector<AssetId> assets;
        for (auto const& camera : ba_input_) {
            // cppcheck-suppress useStlAlgorithm
            assets.push_back(camera.camera_id);
        }

        return assets;
    }  // LCOV_EXCL_LINE

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    // TODO(Jack): Should we actually name this the reference camera? cam0 is  little ambiguous.
    CamStageIds cam0_;
    std::vector<optimization::CameraProblemInput> ba_input_;
    Frames rig_poses_;
    int num_threads_;
    uint64_t approx_sync_delta_ns_;
};

}  // namespace reprojection::steps
