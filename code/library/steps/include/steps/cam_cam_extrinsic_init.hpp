#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// TODO(Jack): The CameraCalibration has way too much information for the extrinsic initialization so we cut it down
// here just to what we need here. But regardless the type handling is ugly and we need to somehow make this more
// explicit!
struct CamCamExtrinsicInitState {
    AssetId camera_id;
    StepId frames_id;

    // NOTE(Jack): This is hardcoded to take the bundle adjustment frames!
    explicit CamCamExtrinsicInitState(CameraCalibration const& camera_calibration)
        : camera_id{camera_calibration.camera_id}, frames_id{camera_calibration.bundle_adjustment_id} {}
};

struct CamCamExtrinsicInit {
    CamCamExtrinsicInit(std::vector<CameraCalibration> const& camera_calibrations, SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicInit; }

    std::vector<AssetId> Assets() const {
        std::vector<AssetId> assets;
        for (auto const& camera : states_) {
            // cppcheck-suppress useStlAlgorithm
            assets.push_back(camera.camera_id);
        }

        return assets;
    }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    // TODO(Jack): Naming!
    std::vector<CamCamExtrinsicInitState> states_;
    // TODO(Jack): Technically we should probably be working with the rig state here, but the rig-state and frame state
    // for the single camera workflows is still interchangeable for now so we ignore it. We will regret it one day :)
    std::map<AssetId, Frames> camera_frames_;
};

}  // namespace reprojection::steps
