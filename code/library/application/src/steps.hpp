

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

struct LinearPoseInitializationStep {
    LinearPoseInitializationStep(CameraInfo const& _camera_info, CameraMeasurements const& _targets,
                                 CameraState const& _camera_state);

    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState camera_state;

    CalibrationStep step_type{CalibrationStep::Lpi};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    Frames Compute() const;

    Frames Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(Frames const& frames, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

}  // namespace reprojection::application
