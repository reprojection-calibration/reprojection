#include "steps/imu_data_loading.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::steps {

ImuDataLoading::ImuDataLoading(AssetId const imu_id, std::string_view serialized_imu_sampler,
                               ImageSampler const& imu_sampler)
    : imu_id_{imu_id}, cache_key_{hashing::HashArguments(serialized_imu_sampler)}, imu_sampler_{imu_sampler} {}

Hash ImuDataLoading::CacheKey(database::CalibrationDatabase& db) const {
    // NOTE(Jack): Just like the image loading the imu data loading does not load anything from the database but instead
    // bootstraps directly from the user/application input.
    static_cast<void>(db);

    return cache_key_;
}

void ImuDataLoading::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    ImuMeasurements imu_data;
    while (auto const data{imu_sampler_()}) {
        auto const& [timestamp_ns, data_i]{*data};

        Array3d const angular_velocity{data_i[0], data_i[1], data_i[2]};
        Array3d const linear_acceleration{data_i[3], data_i[4], data_i[5]};

        imu_data.insert({timestamp_ns, {angular_velocity, linear_acceleration}});
    }

    db.ImuDataInsert(step_id, imu_id_, imu_data);
}

}  // namespace reprojection::steps
