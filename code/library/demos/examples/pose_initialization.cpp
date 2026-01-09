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

    // Cam 0
    // TODO LOAD THE TARGET DIRECTLY INTO THE NEW DATA STRUCTURE
    auto const cam0_targets_stamped{
        database::GetExtractedTargetData(db, sensor_name).value()};  // WARN UNPROTECTED OPTIONAL ACCESS
    for (auto const& target : cam0_targets_stamped) {
        cam_data.frames[target.header.timestamp_ns].extracted_target = target.target;
    }

    calibration::LinearPoseInitialization(InitializationDataView(cam_data));

    // TODO HANDLE POSE DATA AND DATABASE INTERACTIONS NATIVELY WITH A VIEW/THE NEW DATA STRUCT
    std::set<PoseStamped> linear_poses;
    for (auto const& frame_i : cam_data.frames) {
        linear_poses.insert({{frame_i.first, cam_data.sensor.sensor_name}, frame_i.second.initial_pose});
    }
    (void)AddPoseData(linear_poses, database::PoseType::Initial, db);

    // Artificially restrict ourselves to the first couple hundred frames where we converge successfully.
    auto it = cam_data.frames.upper_bound(1520528332714760192);
    cam_data.frames.erase(it, cam_data.frames.end());

    optimization::CameraNonlinearRefinement(OptimizationDataView(cam_data));

    std::set<PoseStamped> optimized_poses;
    for (auto const& frame_i : cam_data.frames) {
        optimized_poses.insert({{frame_i.first, cam_data.sensor.sensor_name},
                                geometry::Log(geometry::Exp(frame_i.second.optimized_pose)
                                                  .inverse())});  // INVERSE ERROR - WHAT IS THE PROPER PLACE TO DO THIS
    }
    (void)AddPoseData(optimized_poses, database::PoseType::Optimized, db);

    std::cout << cam_data.optimized_intrinsics.transpose() << std::endl;

    return EXIT_SUCCESS;
}