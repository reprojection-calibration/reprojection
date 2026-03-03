#include <map>
#include <ranges>

#include "calibration/camera_imu_initialization.hpp"
#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"
#include "optimization/camera_imu_nonlinear_refinement.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "spline/se3_spline.hpp"
#include "spline/spline_initialization.hpp"

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

// WARN(Jack): This is a hack that we need to do so that the spline initialization does not have any massive
// discontinuities or sudden jumps. But there is some bigger problem here that we are missing and need to solve long
// term.
// WARN(Jack): Also note that we do not save aligned_initial_state to the database, we save plain old initial_state and
// use that to calculate the reprojection errors, but use aligned_initial_state to initialize the nonlinear
// optimization. This means that what we are doing here and what we are visualizing in the database are starting to
// diverge. Not nice!
// cppcheck-suppress passedByValue
OptimizationState AlignRotations(OptimizationState state) {
    Vector3d so3_i_1{std::cbegin(state.frames)->second.pose.head<3>()};
    for (auto& frame_i : state.frames | std::views::values) {
        Vector3d so3_i{frame_i.pose.head<3>()};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.pose.head<3>() = so3_i;

        so3_i_1 = so3_i;
    }

    return state;
}

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
    ReprojectionErrors const initial_error{optimization::ReprojectionResiduals(sensor, targets, initial_state)};

    auto const aligned_initial_state{AlignRotations(initial_state)};
    auto const [optimized_state,
                diagnostics]{optimization::CameraNonlinearRefinement(sensor, targets, aligned_initial_state)};
    ReprojectionErrors const optimized_error{optimization::ReprojectionResiduals(sensor, targets, optimized_state)};

    // Write camera initialization to database
    try {
        database::AddCalibrationStep("linear_pose_initialization", db);
        database::AddPoseData(initial_state.frames, "linear_pose_initialization", sensor.sensor_name, db);
        database::AddReprojectionError(initial_error, "linear_pose_initialization", sensor.sensor_name, db);
        database::AddCalibrationStep("nonlinear_refinement", db);
        database::AddPoseData(optimized_state.frames, "nonlinear_refinement", sensor.sensor_name, db);
        database::AddReprojectionError(optimized_error, "nonlinear_refinement", sensor.sensor_name, db);
    } catch (std::exception const& e) {
        std::cout << "Caught " << e.what() << std::endl;
    }

    spline::Se3Spline const interpolated_spline{spline::InitializeSe3SplineState(optimized_state.frames)};
    auto const [poses, errors]{
        optimization::SplineReprojectionResiduals(sensor, targets, optimized_state.camera_state, interpolated_spline)};

    database::AddCalibrationStep("spline_initialization", db);
    database::AddPoseData(poses, "spline_initialization", sensor.sensor_name, db);
    database::AddReprojectionError(errors, "spline_initialization", sensor.sensor_name, db);

    auto const [state, diagnostics2]{
        optimization::SplineNonlinearRefinement(sensor, targets, optimized_state.camera_state, interpolated_spline)};

    auto const [poses1, errors1]{
        optimization::SplineReprojectionResiduals(sensor, targets, state.first, state.second)};


    database::AddCalibrationStep("spline_nonlinear_refinement", db);
    database::AddPoseData(poses1, "spline_nonlinear_refinement", sensor.sensor_name, db);
    database::AddReprojectionError(errors1, "spline_nonlinear_refinement", sensor.sensor_name, db);

    return EXIT_SUCCESS;
}
