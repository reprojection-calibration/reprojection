#include "calibration/camera_imu_initialization.hpp"

#include <ranges>

#include "optimization/angular_velocity_alignment.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/spline_types.hpp"

// TODO(Jack): At multiple places in this file we have to write loops to simply convert the data back and forth between
//  compatible types for the spline and IMU types. Is there anyway that we can unify the representaitons to eliminate
//  these conversion loops everywhere?
// TODO(Jack): Pass VelocityMeasurements and PositionMeasurements directly instead of IMU data to the helpers.

namespace reprojection::calibration {

using namespace spline;

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    Frames const& camera_poses, ImuMeasurements const& imu_data) {
    PositionMeasurements camera_orientations;
    for (auto const& [timestamp_ns, frame_i] : camera_poses) {
        camera_orientations.insert({timestamp_ns, {frame_i.pose.topRows(3)}});
    }
    // TODO(Jack): Is 20 times the number of camera_poses high enough frequency? Do we need to parameterize this?
    CubicBSplineC3 const camera_orientation_spline{
        InitializeC3Spline(camera_orientations, 20 * std::size(camera_poses))};

    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        omega_imu.insert({timestamp_ns, {data_i.angular_velocity}});
    }
    auto const [R_imu_co, diagnostics]{EstimateCameraImuRotation(camera_orientation_spline, omega_imu)};

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
    // This assumes zero translation between camera and imu, of course not true but acceptable for small translations.
    PositionMeasurements so3_co_w;
    for (uint64_t const timestamp_ns : imu_acceleration | std::views::keys) {
        auto const so3_i_co_w{EvaluateSpline<So3Spline>(camera_orientation, timestamp_ns, DerivativeOrder::Null)};
        if (so3_i_co_w.has_value()) {
            so3_co_w.insert({timestamp_ns, {so3_i_co_w.value()}});
        }
    }

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
        // looking vector that is not really 100% gravity.
        double constexpr g{9.80665};                 // LCOV_EXCL_LINE
        return g * net_acceleration_w.normalized();  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::calibration
