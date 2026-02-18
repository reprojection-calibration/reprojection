#include <map>

#include "calibration/linear_pose_initialization.hpp"
#include "calibration_data_views/initialization_view.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
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

    std::string const sensor_name{"/cam0/image_raw"};
    CameraCalibrationData cam_data{
        {sensor_name, CameraModel::DoubleSphere, {0, 512, 0, 512}},
        Array6d{156.82590211, 156.79756958, 250.99978685, 250.9744566, -0.17931409, 0.59133716},
        {},
        {}};

    // Load targets, initialize, and optimize
    database::GetExtractedTargetData(db, cam_data);
    calibration::LinearPoseInitialization(InitializationDataView(cam_data));
    optimization::CameraNonlinearRefinement(OptimizationDataView(cam_data));

    std::cout << cam_data.optimized_intrinsics.transpose() << std::endl;

    // TODO(Jack): The moral of the story here is: all the algorithms use/store the tf_co_w transform. But we need the
    //  tf_w_co for intuitive world frame visualizations and graphs in the dashboard. Therefore we invert the tfs here
    //  before we write them to the database. This is clearly a HACK and we need a real strategy going forward. I guess
    //  this should be centralized in the dashboard data loading logic?
    for (auto& [_, frame_i] : cam_data.frames) {
        if (frame_i.initial_pose.has_value()) {
            frame_i.initial_pose = geometry::Log(geometry::Exp(frame_i.initial_pose.value()).inverse());
        }
        if (frame_i.optimized_pose.has_value()) {
            frame_i.optimized_pose = geometry::Log(geometry::Exp(frame_i.optimized_pose.value()).inverse());
        }
    }

    // Write everything to database
    AddCameraPoseData(cam_data, database::PoseType::Initial, db);
    AddCameraPoseData(cam_data, database::PoseType::Optimized, db);
    database::AddReprojectionError(cam_data, database::PoseType::Initial, db);
    database::AddReprojectionError(cam_data, database::PoseType::Optimized, db);

    return EXIT_SUCCESS;
}