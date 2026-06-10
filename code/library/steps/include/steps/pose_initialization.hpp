#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct PoseInitialization {
    CameraInfo camera_info_;
    CameraMeasurements targets_;
    CameraState intrinsics_;

    CalibrationStep step_type{CalibrationStep::PoseInitialization};

    std::string EntityId() const { return camera_info_.sensor_name; }

    std::string HashInputs() const;

    Frames Compute() const;

    Frames Load(SqlitePtr const db) const;

    void Save(Frames const& initialized_poses, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
