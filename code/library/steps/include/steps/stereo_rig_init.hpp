#pragma once

#include <ranges>

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct StereoRigInit {
    StereoRigInit(std::vector<CamStageIds> const& cams, uint64_t approx_sync_delta_ns, SqlitePtr db);

    // TODO(Jack): Should we rename to reflect "stereo rig init"?
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
    uint64_t approx_sync_delta_ns_;
    // NOTE(Jack): We keep this a vector because we in many places implicitly using the first element as the reference
    // camera. Hopefully can can engineer this away.
    std::vector<AssetId> cam_ids_;
    std::map<AssetId, Frames> cam_frames_;
};

}  // namespace reprojection::steps
