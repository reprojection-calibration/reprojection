#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct ExtrinsicInitStereo {
    ExtrinsicInitStereo(AssetId camera_a_id, StepId camera_a_step, AssetId camera_b_id, StepId camera_b_step,
                        SqlitePtr db);

    static StepType Type() { return StepType::ExtrinsicInit; }

    std::vector<AssetId> Assets() const { return {camera_a_id_, camera_b_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_a_id_;
    StepId camera_a_step_;
    Frames frames_a_;
    AssetId camera_b_id_;
    StepId camera_b_step_;
    Frames frames_b_;
};

}  // namespace reprojection::steps
