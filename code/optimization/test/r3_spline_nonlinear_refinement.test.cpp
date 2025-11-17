#include <gtest/gtest.h>

#include "optimization/nonlinear_refinement.hpp"
#include "spline/r3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

double Squared(double const x) { return x * x; }  // COPY AND PASTED

// TODO(Jack): Unify measurement
using C3Measurement = optimization::C3Measurement;

// TODO(Jack): Unfify test data creation
std::tuple<spline::CubicBSplineC3, std::vector<C3Measurement>> R3SplineOptimizationTestData() {
    // Build spline, we will generate our fake measurements from this for the optimization test.
    std::uint64_t const t0_ns{100};
    std::uint64_t const delta_t_ns{50};
    spline::CubicBSplineC3 spline{t0_ns, delta_t_ns};
    for (auto const& x : std::vector<double>{-2, -1, -0.5, 0.5, 1, 2}) {
        spline.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    std::vector<C3Measurement> measurements;

    // NOTE(Jack): If we do not have any position constraints, and ony velocity or acceleration constraint the
    // optimization will wander aimlessly because the derivatives are the same under any constant offset. Thefore we add
    // one position constraint here. In future problems we might achieve this same thing by settings the first value to
    // zero, of simply fixing it to something.
    auto const position_i{spline::EvaluateSpline<spline::R3Spline>(t0_ns, spline, spline::DerivativeOrder::Null)};
    measurements.push_back(C3Measurement{t0_ns, position_i.value(), spline::DerivativeOrder::Null});

    // For a third degree spline with 6 control points there are three valid time segments! With only four control
    // points there would only be one valid time segment.
    for (size_t i{0}; i < 3 * delta_t_ns; ++i) {
        std::uint64_t const t_i{t0_ns + i};

        auto const velocity_i{spline::EvaluateSpline<spline::R3Spline>(t_i, spline, spline::DerivativeOrder::First)};
        measurements.push_back(C3Measurement{t_i, velocity_i.value(), spline::DerivativeOrder::First});

        auto const acceleration_i{
            spline::EvaluateSpline<spline::R3Spline>(t_i, spline, spline::DerivativeOrder::Second)};
        measurements.push_back(C3Measurement{t_i, acceleration_i.value(), spline::DerivativeOrder::Second});
    }

    return {spline, measurements};
}

TEST(OptimizationR3SplineNonlinearRefinement, TestNoisyR3SplineNonlinearRefinement) {
    auto const [gt_spline, simulated_measurements]{R3SplineOptimizationTestData()};

    // Make a copy and add noise so the optimization has to do some work :)
    spline::CubicBSplineC3 initialization{gt_spline};
    for (size_t i{0}; i < std::size(initialization.control_points); ++i) {
        initialization.control_points[i].array() += 0.2 * i;
    }

    optimization::CubicBSplineC3Refinement handler{initialization};
    for (auto const& measurement : simulated_measurements) {
        bool const success{handler.AddConstraint<spline::R3Spline>(measurement)};
        EXPECT_TRUE(success);  // All simulated measurements come from valid time range
    }

    // Check that we reject a bad measurement point outside the spline domain (bad time)
    bool const success{handler.AddConstraint<spline::R3Spline>({66666, {0, 0, 0}, spline::DerivativeOrder::Null})};
    EXPECT_FALSE(success);

    ceres::Solver::Summary const summary{handler.Solve()};
    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::CubicBSplineC3 const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < std::size(optimized_spline.control_points); ++i) {
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(gt_spline.control_points[i], 1e-6));
    }
}
