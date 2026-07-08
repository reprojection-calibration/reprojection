#include "extrinsic_initialization.hpp"

#include <ranges>

#include "optimization/angular_velocity_alignment.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/physics_constants.hpp"
#include "types/spline_types.hpp"

namespace reprojection::calibration {

using namespace spline;

Vector3d EstimateGravity(CubicBSplineC3 const& so3_spline_w_co, AccelerationMeasurements const& acc_imu,
                         Matrix3d const& R_imu_co) {
    PositionMeasurements aa_w_co;
    for (uint64_t const timestamp_ns : acc_imu | std::views::keys) {
        auto const aa_w_co_i{EvaluateSpline<So3Spline>(so3_spline_w_co, timestamp_ns, DerivativeOrder::Null)};
        if (aa_w_co_i.has_value()) {
            aa_w_co.insert({timestamp_ns, {aa_w_co_i.value()}});
        }
    }

    // Transform the imu acceleration into the world frame and store it. This assumes zero translation between camera
    // and imu, of course not true but is acceptable approximation for small translations.
    MatrixXd acceleration_w(std::size(aa_w_co), 3);
    for (int i{0}; auto const& [timestamp_ns, aa_w_co_i] : aa_w_co) {
        Matrix3d const R_w_co{geometry::Exp(aa_w_co_i.position)};
        Matrix3d const R_co_imu{R_imu_co.inverse()};

        Vector3d const& acc_i_imu{acc_imu.at(timestamp_ns).acceleration};

        Vector3d const acceleration_i_w{R_w_co * R_co_imu * acc_i_imu};
        acceleration_w.row(i) = acceleration_i_w;

        i += 1;
    }

    Vector3d const net_acceleration_w{acceleration_w.colwise().mean()};

    // TODO(Jack): Should this be negative?
    return kGravity * net_acceleration_w.normalized();
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
