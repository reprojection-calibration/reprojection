#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// TODO NAMING!
// TODO SHOULD WE REALLY STORE THE ENTIRE RIG STATE HERE?
// TODO WE SHOULD ALREADY HAVE A STRUCT OR MAP WE CAN USE LIKE THIS RIGHT?
struct CamCamExtrinsicInitState {
    AssetId camera_id;
    StepId frames_id;

    explicit CamCamExtrinsicInitState(CameraCalibration const& camera_calibration)
        : camera_id{camera_calibration.camera_id}, frames_id{camera_calibration.bundle_adjustment_id} {}
};

struct CamCamExtrinsicInit {
    // TODO MAKE THIS TAKE THE CamCamExtrinsicInitState directly, otherwise we pass in too much information!
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
    // TODO NAMING!
    std::vector<CamCamExtrinsicInitState> states_;
    std::map<AssetId, Frames> camera_frames_;
};

}  // namespace reprojection::steps
