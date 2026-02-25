#pragma once

#include <cstdint>

#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

template <typename T>
concept CanEvaluateCubicBSplineC3 =
    requires(Eigen::Ref<Matrix3Kd const> const& P, double const u_i, std::uint64_t const delta_t_ns) {
        // WARN(Jack): I do not think this condition is as strict as it looks! During the transition to using Eigen::Ref
        // this did not catch when the so3 spline still had the old plain const& interface. We should investigate here
        // to make sure we are really doing this right.
        //
        // See note in projection_functions::CanProject why we need this for the Eigen types.
        { P } -> std::same_as<Eigen::Ref<Matrix3Kd const> const&>;

        // TODO(Jack): I would also like to test that u_i and delta_t_ns are the required types shown above, but for
        //  some reason it is not so simple and I cannot find the right way to do it. It is not as simple as just "{ u_i
        //  } -> std::same_as(double const);". What is the right answer, I am not sure!

        { T::template Evaluate<double, DerivativeOrder::Null>(P, u_i, delta_t_ns) } -> std::same_as<Vector3d>;
        { T::template Evaluate<double, DerivativeOrder::First>(P, u_i, delta_t_ns) } -> std::same_as<Vector3d>;
        { T::template Evaluate<double, DerivativeOrder::Second>(P, u_i, delta_t_ns) } -> std::same_as<Vector3d>;
    };

}  // namespace reprojection::spline