#include <map>
#include <ranges>

#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"
#include "geometry/lie.hpp"
#include "optimization/camera_imu_orientation_initialization.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_initialization.hpp"
#include "types/calibration_types.hpp"
#include "types/spline_types.hpp"

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
    Vector3d so3_i_1{std::cbegin(state.frames)->second.pose.topRows(3)};
    for (auto& frame_i : state.frames | std::views::values) {
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

    ImuMeasurements const measured_imu_data{database::GetImuData(db, "/imu0")};

    PositionMeasurements spline_init_positions;
    for (auto const& [timestamp_ns, frame_i] : optimized_state.frames) {
        spline_init_positions.insert({timestamp_ns, {frame_i.pose.topRows(3)}});
    }
    spline::CubicBSplineC3 const so3_spline{
        spline::InitializeC3Spline(spline_init_positions, 20 * std::size(optimized_state.frames))};

    // Imu orientation stuff
    VelocityMeasurements omega_co;
    for (auto const timestamp_ns : measured_imu_data | std::views::keys) {
        auto const omega_co_i{
            spline::EvaluateSpline<spline::So3Spline>(so3_spline, timestamp_ns, spline::DerivativeOrder::First)};

        if (omega_co_i) {
            // HARDCODE SCALE MULTIPLY 1e9
            omega_co.insert({timestamp_ns, {1e9 * omega_co_i.value()}});
        }
    }

    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, imu_data_i] : measured_imu_data) {
        omega_imu.insert({timestamp_ns, {imu_data_i.angular_velocity}});
    }

    auto const [R_imu_co, diagnostics2]{optimization::InitializeCameraImuOrientation(omega_co, omega_imu)};

    ImuMeasurements imu_data;
    for (auto const& [timestamp_ns, omega_co_i] : omega_co) {
        imu_data.insert({timestamp_ns, {R_imu_co * omega_co_i.velocity, Vector3d::Zero()}});
    }

    try {
        database::AddImuData(imu_data, "/interpolated_omega", db);
    } catch (...) {
        std::cout << "\n\tPseudo cache hit\n" << std::endl;
    }

    // Imu gravity stuff
    PositionMeasurements orientations;
    for (auto const timestamp_ns : measured_imu_data | std::views::keys) {
        auto const orientation_i{
            spline::EvaluateSpline<spline::So3Spline>(so3_spline, timestamp_ns, spline::DerivativeOrder::First)};

        if (orientation_i) {
            orientations.insert({timestamp_ns, {orientation_i.value()}});
        }
    }

    ImuMeasurements imu_data_acceleration;
    for (auto const& [timestamp_ns, aa_co_w] : orientations) {
        Matrix3d const R_co_w{geometry::Exp(aa_co_w.position)};
        Vector3d const a_w{R_co_w * R_imu_co.inverse() *
                           measured_imu_data.at(timestamp_ns).linear_acceleration};

        imu_data_acceleration.insert({timestamp_ns, {Vector3d::Zero(),a_w}});
    }

    try {
        database::AddImuData(imu_data_acceleration, "/interpolated_acceleration", db);
    } catch (...) {
        std::cout << "\n\tPseudo cache hit\n" << std::endl;
    }

    return EXIT_SUCCESS;
}