#include "camera_imu_initialization.hpp"

#include <ranges>

#include "optimization/angular_velocity_alignment.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::calibration {

using namespace spline;

// TODO(Jack): Unit test!
Vector3d EstimateGravity(CubicBSplineC3 const& camera_orientation, AccelerationMeasurements const& imu_acceleration,
                         Matrix3d const& R_imu_co) {
    (void)R_imu_co;  // USE!!!!

    // Calculate the camera orientation required to transform each linear acceleration into the camera world frame.
    PositionMeasurements so3_w_b;
    for (uint64_t const timestamp_ns : imu_acceleration | std::views::keys) {
        auto const so3_i_b_w{EvaluateSpline<So3Spline>(camera_orientation, timestamp_ns, DerivativeOrder::Null)};
        if (so3_i_b_w.has_value()) {
            so3_w_b.insert({timestamp_ns, {so3_i_b_w.value()}});
        }
    }

    // Transform the imu acceleration into the world frame and store it. This assumes zero translation between camera
    // and imu, of course not true but is acceptable approximation for small translations.
    MatrixXd acceleration_w(std::size(so3_w_b), 3);
    for (int i{0}; auto const& [timestamp_ns, so3_i_b_w] : so3_w_b) {
        Matrix3d const R_b_w{geometry::Exp(so3_i_b_w.position)};
        Matrix3d const R_w_b{R_b_w};

        Vector3d const& acceleration_i_b{imu_acceleration.at(timestamp_ns).acceleration};

        Vector3d const acceleration_i_w{R_w_b * acceleration_i_b};
        acceleration_w.row(i) = acceleration_i_w;

        i += 1;
    }

    Vector3d const net_acceleration_w{acceleration_w.colwise().mean()};
    double constexpr g{9.80665};

    // TODO(Jack): Should this be negative?
    return g * net_acceleration_w.normalized();
}

VelocityMeasurements ExtractAngularVelocity(ImuMeasurements const& imu_data) {
    VelocityMeasurements imu_angular_velocity;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        imu_angular_velocity.insert({timestamp_ns, {data_i.angular_velocity}});
    }

    return imu_angular_velocity;
}  // LCOV_EXCL_LINE

AccelerationMeasurements ExtractLinearAcceleration(ImuMeasurements const& imu_data) {
    AccelerationMeasurements imu_linear_acceleration;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        imu_linear_acceleration.insert({timestamp_ns, {data_i.linear_acceleration}});
    }

    return imu_linear_acceleration;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::calibration
