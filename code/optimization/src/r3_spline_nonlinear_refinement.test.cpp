#include <gtest/gtest.h>

#include "r3_spline_cost_function.hpp"
#include "spline/r3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

class R3SplineProblemHandler {
   public:
    // TODO(Jack): Is there a way to programmatically force control points and time handler to match/correspond?
    R3SplineProblemHandler(std::vector<Vector3d> const& control_points, spline::TimeHandler const& time_handler)
        : control_points_{control_points}, time_handler_{time_handler} {}

   private:
    std::vector<Vector3d> control_points_;
    spline::TimeHandler time_handler_;
};

}  // namespace reprojection::optimization

using namespace reprojection;

double Squared(double const x) { return x * x; }

TEST(OptimizationR3SplineNonlinearRefinement, TestXxx) {
    std::uint64_t const delta_t_ns{50};
    spline::r3Spline r3_spline{100, delta_t_ns};

    for (auto const& x : std::vector<double>{-1, -0.5, 0.5, 1}) {
        r3_spline.control_points_.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    // Add noise so the optimization has to do some work :)
    std::vector<Vector3d> noisy_init{r3_spline.control_points_};
    for (size_t i{0}; i < std::size(noisy_init); ++i) {
        noisy_init[i].array() += i * 0.1;
    }

    ceres::Problem problem;

    // add one position constraint - otherwise it is not possible to converge!!!!
    auto const position_i{r3_spline.Evaluate(100, spline::DerivativeOrder::Null)};
    ceres::CostFunction* const cost_function_p{
        optimization::CreateR3SplineCostFunction(spline::DerivativeOrder::Null, position_i.value(), 0, delta_t_ns)};
    problem.AddResidualBlock(cost_function_p, nullptr, noisy_init[0].data());

    for (size_t i{0}; i < delta_t_ns; ++i) {
        auto const velocity_i{r3_spline.Evaluate(100 + i, spline::DerivativeOrder::First)};
        auto const acceleration_i{r3_spline.Evaluate(100 + i, spline::DerivativeOrder::Second)};
        if (not velocity_i.has_value() or not acceleration_i.has_value()) {
            std::cout << "Bad spline eval! " << i << std::endl;
            continue;
        }

        // TODO AUTO CALCULATE THIS u_i or package the info in a more accessible way
        double const u_i{static_cast<double>(i) / delta_t_ns};

        ceres::CostFunction* const cost_function_v{optimization::CreateR3SplineCostFunction(
            spline::DerivativeOrder::First, velocity_i.value(), u_i, delta_t_ns)};
        problem.AddResidualBlock(cost_function_v, nullptr, noisy_init[0].data());

        ceres::CostFunction* const cost_function_a{optimization::CreateR3SplineCostFunction(
            spline::DerivativeOrder::Second, acceleration_i.value(), u_i, delta_t_ns)};
        problem.AddResidualBlock(cost_function_a, nullptr, noisy_init[0].data());
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (size_t i{0}; i < std::size(noisy_init); ++i) {
        EXPECT_TRUE(noisy_init[i].isApprox(r3_spline.control_points_[i], 1e-6));
    }
}
