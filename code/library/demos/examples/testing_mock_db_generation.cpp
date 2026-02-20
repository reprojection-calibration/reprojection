#include <map>

#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/aa-testing-mock.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    CameraInfo const sensor{"/mvg_test_data_camera", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [targets, _]{testing_mocks::GenerateMvgData(sensor, gt_intrinsics, 200, 10e9, true)};

    for (auto& [timestamp_ns, target_i] : targets) {
        database::AddImage(timestamp_ns, sensor.sensor_name, db);
        database::AddExtractedTargetData({timestamp_ns, target_i}, sensor.sensor_name, db);
    }

    auto const initial_state{calibration::LinearPoseInitialization(sensor, targets, gt_intrinsics)};
    auto const [optimized_state, diagnostics]{optimization::CameraNonlinearRefinement(sensor, targets, initial_state)};

    AddCameraPoseData(initial_state.frames, sensor.sensor_name, database::PoseType::Initial, db);
    AddCameraPoseData(optimized_state.frames, sensor.sensor_name, database::PoseType::Optimized, db);
    // database::AddReprojectionError(cam_data, database::PoseType::Initial, db);
    // database::AddReprojectionError(cam_data, database::PoseType::Optimized, db);

    ImuMeasurements const data{testing_mocks::GenerateImuData(500, 10e9)};
    for (auto const& sample_i : data) {
        (void)database::AddImuData(sample_i, "/mvg_test_data_imu", db);
    }

    return EXIT_SUCCESS;
}