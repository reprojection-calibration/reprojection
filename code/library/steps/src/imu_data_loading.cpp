#include "steps/imu_data_loading.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::steps {

std::string ImuDataLoading::HashInputs() const { return hashing::HashArguments(cache_key); }

ImuMeasurements ImuDataLoading::Compute() const {
    ImuMeasurements imu_data;
    while (auto const data{imu_data_source()}) {
        auto const& [timestamp_ns, data_i]{*data};

        Array3d const angular_velocity{data_i[0], data_i[1], data_i[2]};
        Array3d const linear_acceleration{data_i[3], data_i[4], data_i[5]};

        imu_data.insert({timestamp_ns, {angular_velocity, linear_acceleration}});
    }

    return imu_data;
}  // LCOV_EXCL_LINE

ImuMeasurements ImuDataLoading::Load(SqlitePtr const db) const { return database::ReadImuData(db, EntityId()); }

void ImuDataLoading::Save(ImuMeasurements const& imu_data, SqlitePtr const db) const {
    database::InsertImuData(db, EntityId(), imu_data);
}

}  // namespace reprojection::steps
