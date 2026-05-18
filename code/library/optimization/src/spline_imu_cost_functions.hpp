#pragma once

#include <ceres/autodiff_cost_function.h>

#include "spline/se3_spline.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

#include "ceres_geometry.hpp"
#include "ceres_kinematics.hpp"

namespace reprojection::optimization {

class SplineImuCostFunction {
   public:
    SplineImuCostFunction(ImuData const& imu_data, double const u_i, uint64_t const delta_t_ns)
        : imu_data_{imu_data}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    // TODO MAKE FULL 6D TF!!!
    template <typename T>
    bool operator()(T const* const tf_imu_co, T const* const gravity_w, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        // Map control point pointers into a usable control point matrix block.
        std::array<T const* const, spline::constants::order> ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                  control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        // FIGURE OUT SCALE!!! 1e9
        Vector3<T> const omega_co{1e9 * spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(
                                            control_points.template topRows<3>(), u_i_, delta_t_ns_)};
        Eigen::Map<Eigen::Vector<T, 6> const> tf_imu_co_XXX(tf_imu_co);  // NAMING!!!!
        Array3<T> const omega_imu{geometry::Exp<T>(tf_imu_co_XXX.template topRows<3>()) * omega_co};

        residual[0] = T(imu_data_.angular_velocity[0]) - omega_imu[0];
        residual[1] = T(imu_data_.angular_velocity[1]) - omega_imu[1];
        residual[2] = T(imu_data_.angular_velocity[2]) - omega_imu[2];

        Vector3<T> const aa_co_w{spline::So3Spline::Evaluate<T, spline::DerivativeOrder::Null>(
            control_points.template topRows<3>(), u_i_, delta_t_ns_)};
        Eigen::Map<Eigen::Vector<T, 3> const> gravity_XXX(gravity_w);
        Vector3<T> const a_w{1e18 * spline::R3Spline::Evaluate<T, spline::DerivativeOrder::Second>(
                                        control_points.template bottomRows<3>(), u_i_, delta_t_ns_)};  // NAMING!
        Array3<T> const a_co{geometry::Exp<T>(aa_co_w.template topRows<3>()) * (gravity_XXX - a_w)};

        // FIGURE OUT SCALE!!! 1e9
        Vector3<T> const alpha_co{1e18 * spline::So3Spline::Evaluate<T, spline::DerivativeOrder::Second>(
                                             control_points.template topRows<3>(), u_i_, delta_t_ns_)};

        Vector3<T> const a_imu{TransformRigidBodyAcceleration<T>(tf_imu_co_XXX, omega_co, alpha_co, a_co)};

        residual[3] = T(imu_data_.linear_acceleration[0]) - a_imu[0];
        residual[4] = T(imu_data_.linear_acceleration[1]) - a_imu[1];
        residual[5] = T(imu_data_.linear_acceleration[2]) - a_imu[2];

        return true;
    }

    static ceres::CostFunction* Create(ImuData const& imu_data, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineImuCostFunction, 6, 6, 3, 6, 6, 6, 6>(
            new SplineImuCostFunction(imu_data, u_i, delta_t_ns));
    }

    ImuData imu_data_;

    double u_i_;
    uint64_t delta_t_ns_;
};

class SplineRotationalVelocityCostFunction {
   public:
    SplineRotationalVelocityCostFunction(Vector3d const& omega, double const u_i, uint64_t const delta_t_ns)
        : omega_{omega}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    // TODO MAKE FULL 6D TF!!!
    template <typename T>
    bool operator()(T const* const aa_imu_co, T const* const control_point_0_ptr, T const* const control_point_1_ptr,
                    T const* const control_point_2_ptr, T const* const control_point_3_ptr, T* const residual) const {
        // Map control point pointers into a usable control point matrix block.
        std::array<T const* const, spline::constants::order> ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                  control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        // FIGURE OUT SCALE!!! 1e9
        Vector3<T> const omega_co{1e9 * spline::So3Spline::Evaluate<T, spline::DerivativeOrder::First>(
                                            control_points.template topRows<3>(), u_i_, delta_t_ns_)};

        Eigen::Map<Eigen::Vector<T, 3> const> aa_imu_co_XXX(aa_imu_co);
        Array3<T> const omega_imu{geometry::Exp<T>(aa_imu_co_XXX) * omega_co};

        residual[0] = T(omega_[0]) - omega_imu[0];
        residual[1] = T(omega_[1]) - omega_imu[1];
        residual[2] = T(omega_[2]) - omega_imu[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& angular_velocity, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineRotationalVelocityCostFunction, 3, 3, 6, 6, 6, 6>(
            new SplineRotationalVelocityCostFunction(angular_velocity, u_i, delta_t_ns));
    }

    Vector3d omega_;

    double u_i_;
    uint64_t delta_t_ns_;
};

class SplineLinearAccelerationCostFunction {
   public:
    SplineLinearAccelerationCostFunction(Vector3d const& acceleration, double const u_i, uint64_t const delta_t_ns)
        : acceleration_{acceleration}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    // TODO MAKE FULL 6D TF!!!
    template <typename T>
    bool operator()(T const* const aa_imu_co, T const* const gravity_w, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        // Map control point pointers into a usable control point matrix block.
        std::array<T const* const, spline::constants::order> ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                  control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        // TODO USE TRANSLATION TOO!
        Array6<T> const aa_co_w{spline::Se3Spline::EvaluatePose<T>(control_points, u_i_, delta_t_ns_)};

        // FIGURE OUT SCALE!!! 1e9
        // TODO FIGURE OUT COORDINATE FRAME!
        Vector3<T> const acceleration{1e18 * spline::R3Spline::Evaluate<T, spline::DerivativeOrder::Second>(
                                                 control_points.template bottomRows<3>(), u_i_, delta_t_ns_)};

        Eigen::Map<Eigen::Vector<T, 3> const> aa_imu_co_XXX(aa_imu_co);
        Eigen::Map<Eigen::Vector<T, 3> const> gravity_XXX(gravity_w);
        Array3<T> const acceleration_imu{geometry::Exp<T>(aa_imu_co_XXX) *
                                         geometry::Exp<T>(aa_co_w.template topRows<3>()) *
                                         (gravity_XXX - acceleration)};

        residual[0] = T(acceleration_[0]) - acceleration_imu[0];
        residual[1] = T(acceleration_[1]) - acceleration_imu[1];
        residual[2] = T(acceleration_[2]) - acceleration_imu[2];

        // TODO(Jack): Add gravity magnitude = 9.8 residual term

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& acceleration, double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineLinearAccelerationCostFunction, 3, 3, 3, 6, 6, 6, 6>(
            new SplineLinearAccelerationCostFunction(acceleration, u_i, delta_t_ns));
    }

    Vector3d acceleration_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace  reprojection::optimization