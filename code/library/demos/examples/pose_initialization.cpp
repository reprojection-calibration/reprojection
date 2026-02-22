#include <map>
#include <ranges>

#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"
#include "geometry/lie.hpp"
#include "optimization/camera_imu_orientation_initialization.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_initialization.hpp"
#include "types/calibration_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

Matrix3d const tf_co_imuxxx{{-0.9995250378696743, 0.029615343885863205, -0.008522328211654736},
                            {0.0075019185074052044, -0.03439736061393144, -0.9993800792498829},
                            {-0.02989013031643309, -0.998969345370175, 0.03415885127385616}};

OptimizationState AlignRotations(OptimizationState state) {
    Vector3d so3_i_1{std::cbegin(state.frames)->second.pose.topRows(3)};
    for (auto& [timestamp_ns, frame_i] : state.frames) {
        Vector3d so3_i{frame_i.pose.topRows(3)};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.pose.topRows(3) = so3_i;

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
    auto [optimized_state,
          diagnostics]{optimization::CameraNonlinearRefinement(sensor, targets, aligned_initial_state)};
    ReprojectionErrors const optimized_error{optimization::ReprojectionResiduals(sensor, targets, optimized_state)};

    // Write camera initialization to database
    try {
        database::AddCameraPoseData(initial_state.frames, sensor.sensor_name, database::PoseType::Initial, db);
        database::AddCameraPoseData(optimized_state.frames, sensor.sensor_name, database::PoseType::Optimized, db);
        database::AddReprojectionError(initial_error, sensor.sensor_name, database::PoseType::Initial, db);
        database::AddReprojectionError(optimized_error, sensor.sensor_name, database::PoseType::Optimized, db);
    } catch (...) {
        std::cout << "\n\tPseudo cache hit\n" << std::endl;
    }

    PositionMeasurements orientations;
    for (auto const& [timestamp_ns, frame_i] : optimized_state.frames) {
        orientations.insert({timestamp_ns, {frame_i.pose.topRows(3)}});
    }
    spline::CubicBSplineC3 const so3_spline{
        spline::InitializeC3Spline(orientations, 20 * std::size(optimized_state.frames))};

    ImuMeasurements const measured_imu_data{database::GetImuData(db, "/imu0")};
    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, imu_data_i] : measured_imu_data) {
        omega_imu.insert({timestamp_ns, {imu_data_i.angular_velocity}});
    }

    VelocityMeasurements omega_co;
    for (auto const timestamp_ns : measured_imu_data | std::views::keys) {
        auto const omega_co_i{
            spline::EvaluateSpline<spline::So3Spline>(so3_spline, timestamp_ns, spline::DerivativeOrder::First)};

        if (omega_co_i) {
            // HARDCODE SCALE MULTIPLY 1e9
            omega_co.insert({timestamp_ns, {1e9 * omega_co_i.value()}});
        }
    }

    auto const [tf, diagnostics2]{optimization::InitializeCameraImuOrientation(omega_co, omega_imu)};

    std::cout << tf << std::endl;
    std::cout << diagnostics2.solver_summary.FullReport() << std::endl;
    ImuMeasurements imu_data;
    for (auto const timestamp_ns : measured_imu_data | std::views::keys) {
        auto const omega_i{
            spline::EvaluateSpline<spline::So3Spline>(so3_spline, timestamp_ns, spline::DerivativeOrder::First)};

        if (omega_i) {
            // HARDCODE SCALE MULTIPLY 1e9
            imu_data.insert({timestamp_ns, {1e9 * tf * omega_i.value(), Vector3d::Zero()}});
        }
    }

    database::AddImuData(imu_data, "/interpolated", db);

    return EXIT_SUCCESS;
}