#pragma once

#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct ImuDataLoading {
    std::string imu_name_;
    std::string serialized_data_signature_;
    ImuSampleSource imu_data_source_;

    static CalibrationStep StepType() { return CalibrationStep::ImuDataLoading; }

    std::string EntityId() const { return imu_name_; }

    std::string HashInputs() const;

    ImuMeasurements Compute() const;

    ImuMeasurements Load(SqlitePtr const db) const;

    void Save(ImuMeasurements const& imu_data, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
