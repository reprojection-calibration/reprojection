#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct IntrinsicInit {
    IntrinsicInit(AssetId camera_id, std::optional<double> const& focal_length, int num_threads, StepId camera_info_id,
                  StepId targets_id, SqlitePtr db);

    static StepType Type() { return StepType::IntrinsicInit; }

    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    std::optional<double> focal_length_;
    int num_threads_;
    CameraInfo camera_info_;
    TargetSamples targets_;
};

}  // namespace reprojection::steps
