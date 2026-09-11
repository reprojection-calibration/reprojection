#pragma once

#include <ranges>

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct CamCamExtrinsicInit {
    CamCamExtrinsicInit(std::vector<CamStageIds> const& camera_calibrations, SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicInit; }

    std::vector<AssetId> Assets() const {
        std::vector<AssetId> assets;
        for (auto const& cam_id : cam_ids_) {
            // cppcheck-suppress useStlAlgorithm
            assets.push_back(cam_id);
        }

        return assets;
    }  // LCOV_EXCL_LINE

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    // NOTE(Jack): We keep this a vector because we in many places implicitly using the first element as the reference
    // camera. Hopefully can can engineer this away.
    std::vector<AssetId> cam_ids_;
    // TODO(Jack): Technically we should probably be working with multiple rig states here, but the rig-state and frame
    // state for the single camera workflows is still interchangeable for now so we ignore it. We will regret it one day
    // :)
    std::map<AssetId, Frames> camera_frames_;
};

}  // namespace reprojection::steps
