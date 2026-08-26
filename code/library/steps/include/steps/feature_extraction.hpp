#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct FeatureExtraction {
    FeatureExtraction(AssetId camera_id, StepId image_loading_id, bool show_extraction, StepId target_info_id,
                      AssetId target_id, SqlitePtr db);

    static StepType Type() { return StepType::FeatureExtraction; }

    // TODO(Jack): We do no strictly need the target asset ID after the constructor is done, so we do not have it as a
    // class member, so we do not write it here with the camera assed ID. Is that ok or are we missing the point?
    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    StepId image_loading_id_;
    bool show_extraction_;
    std::shared_ptr<EncodedImages> images_;
    TargetInfo target_info_;
};

}  // namespace reprojection::steps
