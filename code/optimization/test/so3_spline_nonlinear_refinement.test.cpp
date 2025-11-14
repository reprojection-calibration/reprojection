#include <gtest/gtest.h>

#include "optimization/nonlinear_refinement.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

double Squared(double const x) { return x * x; }  // COPY AND PASTED

using So3Measurement = optimization::So3Measurement;

std::tuple<spline::CubicBSplineC3, std::vector<So3Measurement>> So3SplineOptimizationTestData() {
    std::uint64_t const t0_ns{100};
    std::uint64_t const delta_t_ns{50};
    spline::CubicBSplineC3 spline{t0_ns, delta_t_ns};
    for (auto const& x : std::vector<double>{-1, -0.5, 0.5, 1}) {
        spline.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    std::vector<So3Measurement> measurements;
    for (size_t i{0}; i < delta_t_ns; ++i) {
        std::uint64_t const t_i{t0_ns + i};

        // ERRROR(Jack): How does EvaluateSPline get into this namespace? Does that work differently for templated
        // functions?
        auto const position_i{EvaluateSpline<spline::So3Spline>(t_i, spline, spline::DerivativeOrder::Null)};
        measurements.push_back(So3Measurement{t_i, position_i.value(), spline::DerivativeOrder::Null});

        auto const velocity_i{EvaluateSpline<spline::So3Spline>(t_i, spline, spline::DerivativeOrder::First)};
        measurements.push_back(So3Measurement{t_i, velocity_i.value(), spline::DerivativeOrder::First});

        auto const acceleration_i{
            EvaluateSpline<spline::So3Spline>(t_i, spline, spline::DerivativeOrder::Second)};
        measurements.push_back(So3Measurement{t_i, acceleration_i.value(), spline::DerivativeOrder::Second});
    }

    return {spline, measurements};
}

TEST(OptimizationSo3SplineNonlinearRefinement, TestNoisySo3SplineNonlinearRefinement) {
    auto const [gt_spline, simulated_measurements]{So3SplineOptimizationTestData()};

    spline::CubicBSplineC3 initialization{gt_spline};
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

    spline::CubicBSplineC3 const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < std::size(optimized_spline.control_points); ++i) {
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(gt_spline.control_points[i], 1e-1));
    }
}
