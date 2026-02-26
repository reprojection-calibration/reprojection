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
    Se3Spline(Eigen::Ref<Matrix2NXd const> const& control_points, TimeHandler const& time_handler);

    Se3Spline(std::vector<Vector6d> const& control_points, TimeHandler const& time_handler);

    // TODO(Jack): Are we sure long term that the initialization functions need to return a pair? If they do not then we
    //  can get rid of these, here and for the C3 spline too.
    explicit Se3Spline(std::pair<Matrix2NXd, TimeHandler> const& pair);

    /**
     * \brief Evaluate the spline in non-optimization contexts. If an invalid time is requested std::nullopt is
     * returned.
     */
    std::optional<Vector6d> Evaluate(std::uint64_t const t_ns, DerivativeOrder const derivative) const;

    /**
     * \brief A static function for evaluating the spline pose in optimization cost functions requiring autodiff
     * compatibility.
     *
     * For use in autodiff a function should be templated so that ceres can use the Jet type during auto
     * differentiation, and should ideally not have any branching. Templating is easy, and we achieve "no branching"
     * here by templating/hard coding the calls to *Spline::Evaluate on DerivativeOrder::Null.
     *
     * The semantics of using this function and Se3Spline::Evaluate() are completely different. This is a static
     * function which has no access to the state and requires the user to "feed" it valid control point blocks P and
     * segment normalized time u_i. This happens during the construction of the optimization problem when the cost
     * functions are being initialized and added to the problem.
     */
    template <typename T>
    static Array6<T> EvaluatePose(Matrix2NK<T> const& P, double const u_i, uint64_t const delta_t_ns) {
        assert(0 <= u_i and u_i < 1);
        assert(delta_t_ns > 0);

        constexpr auto D{DerivativeOrder::Null};
        constexpr int N{constants::states};

        Array6<T> pose;
        pose.template head<N>() = So3Spline::Evaluate<T, D>(P.template topRows<N>(), u_i, delta_t_ns);
        pose.template tail<N>() = R3Spline::Evaluate<T, D>(P.template bottomRows<N>(), u_i, delta_t_ns);

        return pose;
    }

    Eigen::Ref<Matrix2NXd const> ControlPoints() const { return control_points_; }

    // WARN(Jack): We tried to return an Eigen::Ref here (and for the R3() method) like we do above for the
    // ControlPoints(), but that does not play nicely with the block expression returned by .topRows()/.bottomRows()! It
    // compiled and executed without segfaults or anything like that, but when we tried to get a map of the local
    // control points in the So3Spline::Evaluate() method, it became clear that the underlying memory was the original
    // row=6 se3 data and not the expected row=3 so3 subset of data. Maybe we need to learn more about how block
    // operations work of the eigen matrix base class, but for now we simply just return a copy of the relevant parts.
    // It is not worth our time to spend any more time looking at this until we prove it's a problem.
    MatrixNXd So3() const { return control_points_.topRows<MatrixNXd::RowsAtCompileTime>(); }

    MatrixNXd R3() const { return control_points_.bottomRows<MatrixNXd::RowsAtCompileTime>(); }

    TimeHandler GetTimeHandler() const { return time_handler_; }

   private:
    Matrix2NXd control_points_;
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline
