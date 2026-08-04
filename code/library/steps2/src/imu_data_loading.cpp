#include "steps/imu_data_loading.hpp"

#include "hashing/hashing.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

ImuDataLoading::ImuDataLoading(AssetId const imu_id, std::string_view serialized_imu_sampler,
                               ImuSampler const& imu_sampler)
    : imu_id_{imu_id}, cache_key_{hashing::HashArguments(serialized_imu_sampler)}, imu_sampler_{imu_sampler} {}

Hash ImuDataLoading::CacheKey() const { return cache_key_; }

void ImuDataLoading::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    ImuMeasurements imu_data;
    while (auto const data{imu_sampler_()}) {
        auto const& [timestamp_ns, data_i]{*data};

        Array3d const angular_velocity{data_i[0], data_i[1], data_i[2]};
        Array3d const linear_acceleration{data_i[3], data_i[4], data_i[5]};

        imu_data.insert({timestamp_ns, {angular_velocity, linear_acceleration}});
    }

    log->info("{{'step_id': {}, 'imu_id': {}, 'num_imu_data': {}}}", step_id.value, imu_id_.value, std::size(imu_data));

    db.ImuDataInsert(step_id, imu_id_, imu_data);
}

}  // namespace reprojection::steps
