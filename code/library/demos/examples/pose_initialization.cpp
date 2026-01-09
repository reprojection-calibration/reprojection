#include <map>

#include "calibration/linear_pose_initialization.hpp"
#include "calibration_data_views/initialization_view.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    std::string const sensor_name{"/cam0/image_raw"};
    CameraCalibrationData cam_data{
        {sensor_name, CameraModel::DoubleSphere},
        Array6d{156.82590211, 156.79756958, 254.99978685, 256.9744566, -0.17931409, 0.59133716},
        {},
        {}};

    // Load targets
    database::GetExtractedTargetData(db, cam_data);

    // Linear initialization and save
    calibration::LinearPoseInitialization(InitializationDataView(cam_data));
    AddPoseData(cam_data, database::PoseType::Initial, db);

    // Artificially restrict ourselves to the first couple hundred frames where we converge successfully.
    auto it = cam_data.frames.upper_bound(1520528332714760192);
    cam_data.frames.erase(it, cam_data.frames.end());

    // Nonlinear optimization and save
    optimization::CameraNonlinearRefinement(OptimizationDataView(cam_data));

    // WARN(Jack): At this time we have unsettled coordinate frame connventions. Because of this we need to invert the
    // poses here to match the initial poses. This is a well known problem!
    for (auto& frame_i : cam_data.frames) {
        frame_i.second.optimized_pose = geometry::Log(geometry::Exp(frame_i.second.optimized_pose).inverse());
    }
    AddPoseData(cam_data, database::PoseType::Optimized, db);

    std::cout << cam_data.optimized_intrinsics.transpose() << std::endl;

    return EXIT_SUCCESS;
}