#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct ExtrinsicInitialization {
    std::string imu_name_;
    std::string camera_name_;
    ImuMeasurements imu_data_;
    // NOTE(Jack): To actually just initialize the cam-imu extrinsics we actually only need the orientation component of
    // the spline. BUT we actually also want to save the ImuErrors to the database as a sort of diagnostic. To do that
    // we need the full spline.
    spline::Se3Spline spline_;

    CalibrationStep step_type{CalibrationStep::ExtrinsicInitialization};

    std::string EntityId() const { return Extrinsic::EntityId(imu_name_, camera_name_); }

    std::string HashInputs() const;

    ImuCamExtrinsic Compute() const;

    ImuCamExtrinsic Load(SqlitePtr const db) const;

    void Save(ImuCamExtrinsic const& extrinsic, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
