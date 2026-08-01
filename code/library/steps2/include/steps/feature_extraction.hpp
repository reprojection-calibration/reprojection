#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct FeatureExtraction {
    FeatureExtraction(AssetId camera, StepId image_loading, bool show_extraction, StepId target_info, AssetId target,
                      database::CalibrationDatabase& db);

    static StepType Type() { return StepType::FeatureExtraction; }

    Hash CacheKey(database::CalibrationDatabase& db) const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_;
    StepId image_loading_;
    bool show_extraction_;
    TargetInfo target_info_;
    std::shared_ptr<EncodedImages> images_;
};

}  // namespace reprojection::steps
