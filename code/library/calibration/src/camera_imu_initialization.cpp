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

    auto const [R_imu_co, diagnostics]{EstimateCameraImuRotation(camera_orientation_spline, imu_data)};
    Vector3d const gravity_w{EstimateGravity(camera_orientation_spline, imu_data, R_imu_co)};

    return {{R_imu_co, diagnostics}, gravity_w};
}

// TODO(Jack): Unit test
std::tuple<Matrix3d, CeresState> EstimateCameraImuRotation(CubicBSplineC3 const& camera_orientation_spline,
                                                           ImuMeasurements const& imu_data) {
    VelocityMeasurements omega_co;
    for (uint64_t const timestamp_ns : imu_data | std::views::keys) {
        if (auto const omega_co_i{
                EvaluateSpline<So3Spline>(camera_orientation_spline, timestamp_ns, DerivativeOrder::First)}) {
            // WARN(Jack): We are hard coding a scale multiplication here of 1e9 to bring it out of ns space and into
            // normal second space. Why we need this I am not 100% sure. But if we do not have it then our spline
            // derivative data does not match the real world scale by exactly a factor of 1e-9 :)
            omega_co.insert({timestamp_ns, {1e9 * omega_co_i.value()}});
        }
    }

    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        omega_imu.insert({timestamp_ns, {data_i.angular_velocity}});
    }

    return optimization::AngularVelocityAlignment(omega_co, omega_imu);
}

// TODO(Jack): Unit test
Vector3d EstimateGravity(CubicBSplineC3 const& camera_orientation_spline, ImuMeasurements const& imu_data,
                         Matrix3d const& R_imu_co) {
    PositionMeasurements aa_co_w;
    for (uint64_t const timestamp_ns : imu_data | std::views::keys) {
        if (auto const orientation_i{
                EvaluateSpline<So3Spline>(camera_orientation_spline, timestamp_ns, DerivativeOrder::Null)}) {
            aa_co_w.insert({timestamp_ns, {orientation_i.value()}});
        }
    }

    MatrixXd a_w(std::size(aa_co_w), 3);
    for (int i{0}; auto const& [timestamp_ns, aa_co_w_i] : aa_co_w) {
        Matrix3d const R_co_w{geometry::Exp(aa_co_w_i.position)};
        Vector3d const a_i_w{R_co_w.inverse() * R_imu_co.inverse() * imu_data.at(timestamp_ns).linear_acceleration};

        a_w.row(i) = a_i_w;

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
    //  might be too strict? Let's try it.
    Vector3d const mean_a_w{a_w.colwise().mean()};
    double constexpr g{9.80665};
    if (mean_a_w.norm() < g) {
        std::cerr << "Setting gravity to zero, are you sure you expected that?" << std::endl;
        return Vector3d::Zero();
    } else {
        return g * mean_a_w.normalized();
    }
}

}  // namespace reprojection::calibration
