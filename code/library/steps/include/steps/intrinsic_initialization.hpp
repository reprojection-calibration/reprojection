#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct IntrinsicInitialization {
    IntrinsicInitialization(AssetId camera_id, int num_threads, StepId camera_info_id, StepId targets_id, SqlitePtr db);

    static StepType Type() { return StepType::IntrinsicInit; }

    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    int num_threads_;
    CameraInfo camera_info_;
    CameraMeasurements targets_;
};

}  // namespace reprojection::steps
