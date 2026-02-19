#include <map>

#include "calibration/linear_pose_initialization.hpp"
#include "calibration_data_views/initialization_view.hpp"
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

    CameraCalibrationData cam_data{testing_mocks::GenerateMvgData(
        200, 10e9, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds, true)};
    for (auto& [timestamp_ns, frame_i] : cam_data.frames) {
        frame_i.initial_pose = std::nullopt;

        FrameHeader const header{timestamp_ns, cam_data.sensor.sensor_name};
        database::AddImage(header, db);
        database::AddExtractedTargetData({header, frame_i.extracted_target}, db);
    }

    calibration::LinearPoseInitialization(InitializationDataView(cam_data));
    optimization::CameraNonlinearRefinement(OptimizationDataView(cam_data));

    AddCameraPoseData(cam_data, database::PoseType::Initial, db);
    AddCameraPoseData(cam_data, database::PoseType::Optimized, db);
    database::AddReprojectionError(cam_data, database::PoseType::Initial, db);
    database::AddReprojectionError(cam_data, database::PoseType::Optimized, db);

    testing_mocks::ImuData const data{testing_mocks::GenerateImuData(500, 10e9)};

    for (auto const& [timestamp_ns, frame_i] : data) {
        FrameHeader const header{timestamp_ns, "/test_imu"};
        (void)database::AddImuData(ImuStamped{header, frame_i}, db);
    }

    return EXIT_SUCCESS;
}