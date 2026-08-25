#pragma once

#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct ImuDataLoading {
    ImuDataLoading(AssetId imu_id, std::string_view serialized_imu_sampler, ImuSampler const& imu_sampler);

    static StepType Type() { return StepType::ImuDataLoading; }

    std::vector<AssetId> Assets() const { return {imu_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId imu_id_;
    Hash cache_key_;
    ImuSampler imu_sampler_;
};

}  // namespace reprojection::steps
