#include "calibration/camera_imu_initialization.hpp"

#include <ranges>

#include "optimization/angular_velocity_alignment.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::calibration {

using namespace spline;

// TODO(Jack): This function is unfortunately primarily a collection of data type conversion loops! Is there any way
//  that we can restructure the data or functions to eliminate these loops?
std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    Frames const& camera_poses, ImuMeasurements const& imu_data) {
    // Data type conversion loop...
    PositionMeasurements camera_orientations;
    for (auto const& [timestamp_ns, frame_i] : camera_poses) {
        camera_orientations.insert({timestamp_ns, {frame_i.pose.topRows(3)}});
    }

    // TODO(Jack): Is 20 times the number of camera_poses high enough frequency? Do we need to parameterize this?
    CubicBSplineC3 const camera_orientation_spline{
        InitializeC3Spline(camera_orientations, 20 * std::size(camera_poses))};

    // Data type conversion loop...
    VelocityMeasurements imu_angular_velocity;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        imu_angular_velocity.insert({timestamp_ns, {data_i.angular_velocity}});
    }

    auto const [R_imu_co, diagnostics]{EstimateCameraImuRotation(camera_orientation_spline, imu_angular_velocity)};

    // Data type conversion loop...
    AccelerationMeasurements imu_linear_acceleration;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        imu_linear_acceleration.insert({timestamp_ns, {data_i.linear_acceleration}});
    }

    Vector3d const gravity_w{EstimateGravity(camera_orientation_spline, imu_linear_acceleration, R_imu_co)};

    return {{R_imu_co, diagnostics}, gravity_w};
}

// TODO(Jack): Unit test
std::tuple<Matrix3d, CeresState> EstimateCameraImuRotation(CubicBSplineC3 const& camera_orientation,
                                                           VelocityMeasurements const& omega_imu) {
    VelocityMeasurements omega_co;
    for (uint64_t const timestamp_ns : omega_imu | std::views::keys) {
        auto const omega_i_co{EvaluateSpline<So3Spline>(camera_orientation, timestamp_ns, DerivativeOrder::First)};
        if (omega_i_co.has_value()) {
            // WARN(Jack): We are hard coding a scale multiplication here of 1e9 to bring it out of ns space and into
            // normal second space. Why we need this I am not 100% sure. But if we do not have it then our spline
            // derivative data does not match the real world scale by exactly a factor of 1e-9 :)
            omega_co.insert({timestamp_ns, {1e9 * omega_i_co.value()}});
        }
    }

    return optimization::AngularVelocityAlignment(omega_co, omega_imu);
}

// TODO(Jack): Unit test
Vector3d EstimateGravity(CubicBSplineC3 const& camera_orientation, AccelerationMeasurements const& imu_acceleration,
                         Matrix3d const& R_imu_co) {
    // Calculate the camera orientation required to transform each linear acceleration into the camera world frame.
    PositionMeasurements so3_co_w;
    for (uint64_t const timestamp_ns : imu_acceleration | std::views::keys) {
        auto const so3_i_co_w{EvaluateSpline<So3Spline>(camera_orientation, timestamp_ns, DerivativeOrder::Null)};
        if (so3_i_co_w.has_value()) {
            so3_co_w.insert({timestamp_ns, {so3_i_co_w.value()}});
        }
    }

    // Transform the imu acceleration into the world frame and store it. This assumes zero translation between camera
    // and imu, of course not true but is acceptable approximation for small translations.
    MatrixXd acceleration_w(std::size(so3_co_w), 3);
    for (int i{0}; auto const& [timestamp_ns, so3_i_co_w] : so3_co_w) {
        Matrix3d const R_w_co{geometry::Exp(so3_i_co_w.position).inverse()};
        Matrix3d const R_co_imu{R_imu_co.inverse()};
        Vector3d const& acceleration_i_imu{imu_acceleration.at(timestamp_ns).acceleration};

        Vector3d const acceleration_i_w{R_w_co * R_co_imu * acceleration_i_imu};
        acceleration_w.row(i) = acceleration_i_w;

        i += 1;
    }

    // NOTE(Jack): This logic here is handling the case where there is no gravity in the acceleration data and
    // protecting us against a division by zero. How can this happen?
    //      1) The imu is in free fall -_-, very unlikely!
    //      2) The imu has some filtering which optimizes and subtracts gravity - possible.
    //      3) The imu test mock data does not have gravity added to the acceleration data :)
    //
    // TODO(Jack): What is the proper threshold to test here? Technically the acceleration vector should roughly have
    //  length 9.8. But what is actually reasonable here is not clear to me. A long term strategy is needed here and
    //  should come as we see more data. My gut tells me that the actual threshold would be to say less than g. But that
    //  might be too strict?
    Vector3d const net_acceleration_w{acceleration_w.colwise().mean()};
    if (net_acceleration_w.norm() < 1) {
        return Vector3d::Zero();
    } else {
        // TODO(Jack): Engineer more sophisticated IMU test data with gravity so that we can cover this branch in unit
        //  testing!
        // NOTE(Jack): By normalizing and then multiplying by 9.81 we are hacking this/stuffing this into a gravity
        // looking vector that is not really 100% gravity because it includes non-zero mean components like noise, bias,
        // and any non-symmetric accelerations (ex. free fall).
        double constexpr g{9.80665};                 // LCOV_EXCL_LINE
        return g * net_acceleration_w.normalized();  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::calibration
