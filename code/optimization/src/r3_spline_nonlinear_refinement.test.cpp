#include <gtest/gtest.h>

#include "r3_spline_cost_function.hpp"
#include "spline/r3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

class R3SplineProblemHandler {
   public:
    explicit R3SplineProblemHandler(spline::R3SplineState const& spline) : spline_{spline} {}

    // TODO(Jack): Should we have a type that makes the relationship between the time and the position explicit? I mean
    // they are really one thing!
    // NOTE(Jack): We will keep this as no discard because I want to force the user to responsibly handle invalid
    // conditions when adding constraints.
    [[nodiscard]] bool AddConstraint(std::uint64_t const t_ns, Vector3d const& r3_position,
                                     spline::DerivativeOrder const order) {
        auto const normalized_position{spline_.time_handler.SplinePosition(t_ns, std::size(spline_.control_points))};
        if (not normalized_position.has_value()) {
            return false;
        }
        auto const [u_i, i]{normalized_position.value()};

        ceres::CostFunction* const cost_function{
            optimization::CreateR3SplineCostFunction(order, r3_position, u_i, spline_.time_handler.delta_t_ns_)};

        problem_.AddResidualBlock(cost_function, nullptr, spline_.control_points[i].data());

        return true;
    }

    spline::R3SplineState GetSpline() { return spline_; }

    // TODO(Jack): Should these be private? Or is ok to have them public?
    // TODO(Jack): Is the problem mutated by ceres solving? Or can it be const?
    // TODO(Jack): What are the state semantics here? What if we returned a copy or something like that, are we toast?
    ceres::Problem problem_;

   private:
    spline::R3SplineState spline_;
};

}  // namespace reprojection::optimization

using namespace reprojection;

double Squared(double const x) { return x * x; }

TEST(OptimizationR3SplineNonlinearRefinement, TestXxx) {
    std::uint64_t const delta_t_ns{50};
    spline::R3SplineState gt_r3_spline{100, delta_t_ns};
    for (auto const& x : std::vector<double>{-1, -0.5, 0.5, 1}) {
        gt_r3_spline.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    // Make a copy and add noise so the optimization has to do some work :)
    spline::R3SplineState initialization{gt_r3_spline};
    for (size_t i{0}; i < std::size(initialization.control_points); ++i) {
        initialization.control_points[i].array() += 0.1 * i;
    }

    optimization::R3SplineProblemHandler handler{initialization};

    // add one position constraint - otherwise it is not possible to converge!!!!
    auto const position_i{EvaluateR3(100, gt_r3_spline, spline::DerivativeOrder::Null)};
    if (not position_i.has_value()) {
        std::cout << "Bad spline eval! " << std::endl;
        exit(1);
    }
    bool success{handler.AddConstraint(100, position_i.value(), spline::DerivativeOrder::Null)};
    ASSERT_TRUE(success);

    for (size_t i{0}; i < delta_t_ns; ++i) {
        std::uint64_t const t_i{100 + i};

        // Create fake measurement data by evaluating the gt spline
        auto const velocity_i{EvaluateR3(t_i, gt_r3_spline, spline::DerivativeOrder::First)};
        auto const acceleration_i{EvaluateR3(t_i, gt_r3_spline, spline::DerivativeOrder::Second)};
        if (not velocity_i.has_value() or not acceleration_i.has_value()) {
            std::cout << "Bad spline eval! " << i << std::endl;
            continue;
        }

        success = handler.AddConstraint(t_i, velocity_i.value(), spline::DerivativeOrder::First);
        ASSERT_TRUE(success);
        success = handler.AddConstraint(t_i, acceleration_i.value(), spline::DerivativeOrder::Second);
        ASSERT_TRUE(success);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &handler.problem_, &summary);

    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::R3SplineState const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < std::size(optimized_spline.control_points); ++i) {
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(gt_r3_spline.control_points[i], 1e-6));
    }
}
