#include <gtest/gtest.h>

#include "optimization/nonlinear_refinement.hpp"
#include "so3_spline_cost_function.hpp"
#include "spline/so3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

class So3SplineNonlinearRefinement {
   public:
    explicit So3SplineNonlinearRefinement(spline::So3SplineState const& spline) : spline_{spline} {}

    [[nodiscard]] bool AddConstraint(R3Measurement const& constraint) {
        auto const normalized_position{
            spline_.time_handler.SplinePosition(constraint.t_ns, std::size(spline_.control_points))};
        if (not normalized_position.has_value()) {
            return false;
        }
        auto const [u_i, i]{normalized_position.value()};

        ceres::CostFunction* const cost_function{optimization::CreateSo3SplineCostFunction(
            constraint.type, constraint.r3, u_i, spline_.time_handler.delta_t_ns_)};

        problem_.AddResidualBlock(cost_function, nullptr, spline_.control_points[i].data(),
                                  spline_.control_points[i + 1].data(), spline_.control_points[i + 2].data(),
                                  spline_.control_points[i + 3].data());

        return true;
    }

    ceres::Solver::Summary Solve() {
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem_, &summary);

        return summary;
    }

    spline::So3SplineState GetSpline() const { return spline_; }

   private:
    spline::So3SplineState spline_;  // Stores the state we are optimizing
    ceres::Problem problem_;
};

}  // namespace reprojection::optimization

using namespace reprojection;

double Squared(double const x) { return x * x; }  // COPY AND PASTED

using R3Measurement = optimization::R3Measurement;

std::tuple<spline::So3SplineState, std::vector<R3Measurement>> So3SplineOptimizationTestData() {
    std::uint64_t const t0_ns{100};
    std::uint64_t const delta_t_ns{50};
    spline::So3SplineState spline{100, delta_t_ns};
    for (auto const& x : std::vector<double>{-2, -1, -0.5, 0.5, 1, 2}) {
        spline.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    std::vector<R3Measurement> measurements;

    auto const position_i{EvaluateSo3(t0_ns, spline, spline::DerivativeOrder::Null)};
    measurements.push_back(R3Measurement{t0_ns, position_i.value(), spline::DerivativeOrder::Null});

    for (size_t i{0}; i < 3 * delta_t_ns; ++i) {
        std::uint64_t const t_i{100 + i};

        auto const velocity_i{EvaluateSo3(t_i, spline, spline::DerivativeOrder::First)};
        measurements.push_back(R3Measurement{t_i, velocity_i.value(), spline::DerivativeOrder::First});

        auto const acceleration_i{EvaluateSo3(t_i, spline, spline::DerivativeOrder::Second)};
        measurements.push_back(R3Measurement{t_i, acceleration_i.value(), spline::DerivativeOrder::Second});
    }

    return {spline, measurements};
}

TEST(OptimizationSo3SplineNonlinearRefinement, TestNoisySo3SplineNonlinearRefinement) {
    auto const [gt_spline, simulated_measurements]{So3SplineOptimizationTestData()};

    spline::So3SplineState initialization{gt_spline};
    for (size_t i{0}; i < std::size(initialization.control_points); ++i) {
        initialization.control_points[i].array() += 0.2 * i;
    }

    optimization::So3SplineNonlinearRefinement handler{initialization};
    for (auto const& measurement : simulated_measurements) {
        bool const success{handler.AddConstraint(measurement)};
        EXPECT_TRUE(success);
    }

    bool const success{handler.AddConstraint({66666, {0, 0, 0}, spline::DerivativeOrder::Null})};
    EXPECT_FALSE(success);

    ceres::Solver::Summary const summary{handler.Solve()};
    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::So3SplineState const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < std::size(optimized_spline.control_points); ++i) {
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(gt_spline.control_points[i], 1e-6));
    }
}
