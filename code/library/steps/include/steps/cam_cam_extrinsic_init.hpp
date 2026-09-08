#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct CamCamExtrinsicInit {
    CamCamExtrinsicInit(std::vector<CameraCalibration> const& camera_calibrations, SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicInit; }

    std::vector<AssetId> Assets() const {
        std::vector<AssetId> assets;
        for (auto const& camera : camera_calibrations_) {
            // cppcheck-suppress useStlAlgorithm
            assets.push_back(camera.camera_id);
        }

        return assets;
    }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    std::vector<CameraCalibration> camera_calibrations_;
    std::map<AssetId, Frames> camera_frames_;
};

}  // namespace reprojection::steps
