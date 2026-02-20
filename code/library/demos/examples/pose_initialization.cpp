#include <map>

#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "geometry/lie.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    CameraInfo const sensor{"/cam0/image_raw", CameraModel::DoubleSphere, {0, 512, 0, 512}};
    CameraState const intrinsics{
        Array6d{156.82590211, 156.79756958, 250.99978685, 250.9744566, -0.17931409, 0.59133716}};

    // Load targets, initialize, and optimize
    CameraMeasurements const targets{database::GetExtractedTargetData(db, sensor.sensor_name)};

    auto const initial_state{calibration::LinearPoseInitialization(sensor, targets, intrinsics)};
    auto const [optimized_state, diagnostics]{optimization::CameraNonlinearRefinement(sensor, targets, initial_state)};

    // Write everything to database
    AddCameraPoseData(initial_state.frames, sensor.sensor_name, database::PoseType::Initial, db);
    AddCameraPoseData(optimized_state.frames, sensor.sensor_name, database::PoseType::Optimized, db);
    // database::AddReprojectionError(cam_data, database::PoseType::Initial, db);
    // database::AddReprojectionError(cam_data, database::PoseType::Optimized, db);

    std::cout << optimized_state.camera_state.intrinsics.transpose() << std::endl;

    return EXIT_SUCCESS;
}