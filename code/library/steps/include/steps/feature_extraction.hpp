#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"

namespace reprojection::steps {

struct FeatureExtraction {
    FeatureExtraction(AssetId camera_id, StepId image_loading_id, bool show_extraction, StepId target_info_id,
                      AssetId target_id, database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::FeatureExtraction; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase const& db) const;

   private:
    AssetId camera_id_;
    StepId image_loading_id_;
    bool show_extraction_;
    TargetInfo target_info_;
    std::shared_ptr<EncodedImages> images_;
};

}  // namespace reprojection::steps
