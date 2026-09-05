#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

// TODO FILE NAMING!!!
// TODO FILE NAMING!!!
// TODO FILE NAMING!!!
// TODO FILE NAMING!!!
// TODO JACK CAN THIS JUST ONE DAY BE THE BUNDLE ADJUSTMENT STEP ITSELF? THERE IS NOT SO MUCH DIFFERENCE HONESTLY?

namespace reprojection::steps {

struct ExtrinsicCamOptimization {
    ExtrinsicCamOptimization(std::vector<CameraCalibration> const& camera_calibrations, SqlitePtr db);

    static StepType Type() {
        return
            // TODO DO WE NEED A NEW TYPE FOR THE CAM EXTRINSIC?
            StepType::ExtrinsicOptimization;
    }

    std::vector<AssetId> Assets() const {
        // TODO(Jack): Do we also need to add the target id?
        std::vector<AssetId> assets;
        for (auto const& camera : cameras_) {
            assets.push_back(camera.camera_id);
        }

        return assets;
    }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    // TODO TODO THIS NAME IS WAY TOO GENERIC! How many different types of "camera" do we have at this point....?
    CameraCalibration reference_camera_;
    std::vector<CameraProblemInput> cameras_;
    Frames rig_poses_;
};

}  // namespace reprojection::steps
