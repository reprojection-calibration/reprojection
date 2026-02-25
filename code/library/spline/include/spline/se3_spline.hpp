#pragma once

#include <optional>

#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(TimeHandler const& time_handler, Eigen::Ref<Matrix2NXd const> const& control_points);

    Se3Spline(TimeHandler const& time_handler, std::vector<Vector6d> const& control_points);

    std::optional<Vector6d> Evaluate(std::uint64_t const t_ns,
                                     DerivativeOrder const derivative = DerivativeOrder::Null) const;

    // TODO TEST!!!
    // NOTE(Jack): We need a fully compile time determined (i.e. we hardcode DerivativeOrder::Null below) templated
    // wrapper like this for use in the ceres spline pose optimization autodiff. If it turns out that we need other
    // templated specializations of the full se3 evaluation than maybe we need to refactor this, but if it is just the
    // pose then this is fine!
    template <typename T>
    static Array6<T> EvaluatePose(Matrix2NK<T> const& P, double const u_i, uint64_t const delta_t_ns) {
        assert(0 <= u_i and u_i < 1);
        assert(delta_t_ns > 0);

        constexpr DerivativeOrder null{DerivativeOrder::Null};  // Position evaluation order
        constexpr int N{constants::states};

        Array6<T> pose;
        pose.template head<N>() = So3Spline::Evaluate<T, null>(P.template topRows<N>(), u_i, delta_t_ns);
        pose.template tail<N>() = R3Spline::Evaluate<T, null>(P.template bottomRows<N>(), u_i, delta_t_ns);

        return pose;
    }

   private:
    // TODO(Jack): We have the time handler duplicated inside of these two splines. It would be nicer if somehow we
    //  could just store the control points in one big matrix (Matrix2NXd) and then one common time handler that is then
    //  passed off to the evaluation functions for each respective spline.
    CubicBSplineC3 so3_spline_;
    CubicBSplineC3 r3_spline_;
};

}  // namespace reprojection::spline
