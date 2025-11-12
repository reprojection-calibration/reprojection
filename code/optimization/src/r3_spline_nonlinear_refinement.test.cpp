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

        problem_.AddResidualBlock(cost_function, nullptr, spline_.control_points[i].data(),
                                  spline_.control_points[i + 1].data(), spline_.control_points[i + 2].data(),
                                  spline_.control_points[i + 3].data());

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

// TODO USE THIS AND MAKE ORDER CONSISENT WITH THE HANDLER
struct R3Measurement {
    Vector3d r3;
    std::uint64_t t_ns;
    spline::DerivativeOrder type;
};

std::tuple<spline::R3SplineState, std::vector<R3Measurement>> R3SplineOptimizationTestData() {
    // Build spline, we will generate our fake measurements from this, and later add noise to the control points to
    // simulate a noisy initialization.
    std::uint64_t const t0_ns{100};
    std::uint64_t const delta_t_ns{50};
    spline::R3SplineState gt_r3_spline{100, delta_t_ns};
    for (auto const& x : std::vector<double>{-2, -1, -0.5, 0.5, 1, 2}) {
        gt_r3_spline.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    std::vector<R3Measurement> measurements;

    // NOTE(Jack): If we do not have any position constraints, and ony velocity or acceleration constraint the
    // optimization will wander aimlessly because the derivatives are the same under any constant offset. Thefore we add
    // one position constraint here. In future problems we might achieve this same thing by settings the first value to
    // zero, of simply fixing it to something.
    auto const position_i{EvaluateR3(t0_ns, gt_r3_spline, spline::DerivativeOrder::Null)};
    measurements.push_back(R3Measurement{position_i.value(), t0_ns, spline::DerivativeOrder::Null});

    for (size_t i{0}; i < 3 * delta_t_ns; ++i) {
        std::uint64_t const t_i{100 + i};

        auto const velocity_i{EvaluateR3(t_i, gt_r3_spline, spline::DerivativeOrder::First)};
        measurements.push_back(R3Measurement{velocity_i.value(), t_i, spline::DerivativeOrder::First});

        auto const acceleration_i{EvaluateR3(t_i, gt_r3_spline, spline::DerivativeOrder::Second)};
        measurements.push_back(R3Measurement{acceleration_i.value(), t_i, spline::DerivativeOrder::Second});
    }

    return {gt_r3_spline, measurements};
}

TEST(OptimizationR3SplineNonlinearRefinement, TestXxx) {
    auto const [gt_spline, simulated_measurements]{R3SplineOptimizationTestData()};

    // Make a copy and add noise so the optimization has to do some work :)
    spline::R3SplineState initialization{gt_spline};
    for (size_t i{0}; i < std::size(initialization.control_points); ++i) {
        initialization.control_points[i].array() += 0.2 * i;
    }

    optimization::R3SplineProblemHandler handler{initialization};
    for (auto const& measurement : simulated_measurements) {
        bool const success{handler.AddConstraint(measurement.t_ns, measurement.r3, measurement.type)};
        EXPECT_TRUE(success);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &handler.problem_, &summary);

    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::R3SplineState const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < std::size(optimized_spline.control_points); ++i) {
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(gt_spline.control_points[i], 1e-6));
    }
}
