#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct FeatureExtraction {
    FeatureExtraction(AssetId camera_id, std::string_view serialized_image_sampler, ImageSampler const& image_sampler,
                      bool show_extraction, StepId target_info_id, AssetId target_id, SqlitePtr db);

    static StepType Type() { return StepType::FeatureExtraction; }

    // TODO(Jack): We do no strictly need the target asset ID after the constructor is done, so we do not have it as a
    // class member, so we do not write it here with the camera assed ID. Is that ok or are we missing the point?
    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    Hash image_sampler_hash_;
    ImageSampler image_sampler_;
    bool show_extraction_;
    TargetInfo target_info_;
};

}  // namespace reprojection::steps
