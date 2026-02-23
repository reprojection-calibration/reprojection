#include "calibration/camera_imu_extrinsic_initialization.hpp"

#include <ranges>

#include "optimization/camera_imu_orientation_initialization.hpp"
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

// TODO NAMING
std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> CameraImuExtrinsicInitialization(
    Frames const& camera_poses, ImuMeasurements const& imu_data) {
    PositionMeasurements aa_co_w;
    for (auto const& [timestamp_ns, frame_i] : camera_poses) {
        aa_co_w.insert({timestamp_ns, {frame_i.pose.topRows(3)}});
    }
    // TODO(Jack): Is 20 times the number of camera_poses high enough frequency? Do we need to parameterize this?
    CubicBSplineC3 const so3_spline{InitializeC3Spline(aa_co_w, 20 * std::size(camera_poses))};

    // TODO(Jack): Do we need these diagnostics actually? How will the user use this?
    auto const [R_imu_co, diagnostics]{InitializeCameraImuRotation(so3_spline, imu_data)};
    Vector3d const gravity_w{InitializeGravity(so3_spline, imu_data, R_imu_co)};

    return {{R_imu_co, diagnostics}, gravity_w};
}

// TODO NAMING
std::tuple<Matrix3d, CeresState> InitializeCameraImuRotation(CubicBSplineC3 const& so3_spline,
                                                             ImuMeasurements const& imu_data) {
    VelocityMeasurements omega_co;
    for (uint64_t const timestamp_ns : imu_data | std::views::keys) {
        if (auto const omega_co_i{EvaluateSpline<So3Spline>(so3_spline, timestamp_ns, DerivativeOrder::First)}) {
            // WARN(Jack): We are hard coding a scale multiplication here of 1e9 to bring it out of ns space and into
            // normal second space. Why we need this I am not 100% sure.
            omega_co.insert({timestamp_ns, {1e9 * omega_co_i.value()}});
        }
    }

    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, imu_data_i] : imu_data) {
        omega_imu.insert({timestamp_ns, {imu_data_i.angular_velocity}});
    }

    return optimization::InitializeCameraImuOrientation(omega_co, omega_imu);
}

// TODO NAMING!!!!
// NOTE(Jack): This function is based on a simple assumption, and that is the sum of all accelerations induced by the
// user during a calibration data collection will be zero. Essentially the user is making a bunch of symmetric
// repetitive motions (ex. moving the sensor left-right, up-down etc.) and therefore the accelerations should also be
// symmetric and cancel out when summed.
//
// Therefore, when we sum all the accelerations, the symmetric motion generated accelerations will disappear and only
// gravity, which is asymmetric, will remain. Of course this might not work if the accelerometer has a large bias or the
// user does not perform symmetric motions (ex. repeatedly moves quickly to the right and then slowly back to the left).
Vector3d InitializeGravity(CubicBSplineC3 const& so3_spline, ImuMeasurements const& imu_data,
                           Matrix3d const& R_imu_co) {
    PositionMeasurements aa_co_w;
    for (uint64_t const timestamp_ns : imu_data | std::views::keys) {
        if (auto const orientation_i{EvaluateSpline<So3Spline>(so3_spline, timestamp_ns, DerivativeOrder::Null)}) {
            aa_co_w.insert({timestamp_ns, {orientation_i.value()}});
        }
    }

    // TODO THIS ASSUMES SAME POSITION! Just like the orientation init right?
    MatrixXd a_w(std::size(aa_co_w), 3);
    for (int i{0}; auto const& [timestamp_ns, aa_co_w_i] : aa_co_w) {
        Matrix3d const R_co_w{geometry::Exp(aa_co_w_i.position)};
        Vector3d const a_w_i{R_co_w.inverse() * R_imu_co.inverse() * imu_data.at(timestamp_ns).linear_acceleration};

        a_w.row(i) = a_w_i;

        i += 1;
    }

    double constexpr g{9.80665};
    Vector3d const mean_a_w{a_w.colwise().mean()};

    return g * mean_a_w.normalized();
}

}  // namespace reprojection::calibration
