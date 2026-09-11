#pragma once

#include "optimization/extrinsic_optimization.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

// TODO(Jack): One day can we just have a single bundle adjustment step that generically does a "rig" whether it is
// single or multi-cam, I think we are not too far away.

namespace reprojection::steps {

struct CamCamExtrinsicOptimization {
    CamCamExtrinsicOptimization(std::vector<CameraCalibration> const& camera_calibrations, StepId extrinsic_init_id,
                                SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicOptimization; }

    std::vector<AssetId> Assets() const {
        std::vector<AssetId> assets;
        for (auto const& camera : cameras_) {
            // cppcheck-suppress useStlAlgorithm
            assets.push_back(camera.camera_id);
        }

        return assets;
    }  // LCOV_EXCL_LINE

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    // TODO(Jack): We really have wayyy too many types of "camera". Is there really nothing we can do better here?
    CameraCalibration reference_camera_;
    std::vector<optimization::CameraProblemInput> cameras_;
    Frames rig_poses_;
};

}  // namespace reprojection::steps
