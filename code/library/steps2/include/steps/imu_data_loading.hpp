#pragma once

#include "database/calibration_database.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct ImuDataLoading {
    ImuDataLoading(AssetId imu_id, std::string_view serialized_imu_sampler, ImuSampler const& imu_sampler);

    static StepType Type() { return StepType::ImuDataLoading; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId imu_id_;
    Hash cache_key_;
    ImuSampler imu_sampler_;
};

}  // namespace reprojection::steps
