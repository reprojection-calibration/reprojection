#pragma once

#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct ImuDataLoading {
    std::string sensor_name_;
    std::string cache_key_;
    ImuDataSourceSignature imu_data_source_;

    CalibrationStep step_type{CalibrationStep::ImuDataLoading};

    std::string EntityId() const { return sensor_name_; }

    std::string HashInputs() const;

    ImuMeasurements Compute() const;

    ImuMeasurements Load(SqlitePtr const db) const;

    void Save(ImuMeasurements const& imu_data, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
