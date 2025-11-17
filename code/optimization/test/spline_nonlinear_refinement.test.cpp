#include <gtest/gtest.h>

#include "optimization/nonlinear_refinement.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

class OptimizationSplineNonlinearRefinementFixture : public ::testing::Test {
   protected:
    OptimizationSplineNonlinearRefinementFixture() {
        for (auto const& x : std::vector<double>{-2, -1, -0.5, 0.5, 1, 2}) {
            spline_.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
        }
    }

    template <typename T_Model>
    // REQUIRE CONCEPT
    std::vector<optimization::C3Measurement> CalculateMeasurements() {
        std::vector<optimization::C3Measurement> measurements;

        // Given five control points we get three valid time segments of length delta_t_ns_
        for (size_t i{0}; i < 3 * delta_t_ns_; ++i) {
            std::uint64_t const t_i{t0_ns_ + i};

            auto const position_i{spline::EvaluateSpline<T_Model>(t_i, spline_, spline::DerivativeOrder::Null)};
            measurements.push_back(optimization::C3Measurement{t_i, position_i.value(), spline::DerivativeOrder::Null});

            auto const velocity_i{spline::EvaluateSpline<T_Model>(t_i, spline_, spline::DerivativeOrder::First)};
            measurements.push_back(
                optimization::C3Measurement{t_i, velocity_i.value(), spline::DerivativeOrder::First});

            auto const acceleration_i{spline::EvaluateSpline<T_Model>(t_i, spline_, spline::DerivativeOrder::Second)};
            measurements.push_back(
                optimization::C3Measurement{t_i, acceleration_i.value(), spline::DerivativeOrder::Second});
        }

        return measurements;
    }

    std::uint64_t t0_ns_{100};
    std::uint64_t delta_t_ns_{50};
    spline::CubicBSplineC3 spline_{t0_ns_, delta_t_ns_};

   private:
    double static Squared(double const x) { return x * x; }
};

// REMOVE
double Squared(double const x) { return x * x; }  // COPY AND PASTED

// MOVE
using C3Measurement = optimization::C3Measurement;

std::tuple<spline::CubicBSplineC3, std::vector<C3Measurement>> R3SplineOptimizationTestData() {
    // Build spline, we will generate our fake measurements from this for the optimization test.
    std::uint64_t const t0_ns{100};
    std::uint64_t const delta_t_ns{50};
    spline::CubicBSplineC3 spline{t0_ns, delta_t_ns};
    for (auto const& x : std::vector<double>{-2, -1, -0.5, 0.5, 1, 2}) {
        spline.control_points.push_back(Vector3d{x, Squared(x), Squared(x)});
    }

    std::vector<C3Measurement> measurements;

    // For a third degree spline with 6 control points there are three valid time segments! With only four control
    // points there would only be one valid time segment.
    for (size_t i{0}; i < 3 * delta_t_ns; ++i) {
        std::uint64_t const t_i{t0_ns + i};

        auto const position_i{spline::EvaluateSpline<spline::R3Spline>(t_i, spline, spline::DerivativeOrder::Null)};
        measurements.push_back(C3Measurement{t_i, position_i.value(), spline::DerivativeOrder::Null});

        auto const velocity_i{spline::EvaluateSpline<spline::R3Spline>(t_i, spline, spline::DerivativeOrder::First)};
        measurements.push_back(C3Measurement{t_i, velocity_i.value(), spline::DerivativeOrder::First});

        auto const acceleration_i{
            spline::EvaluateSpline<spline::R3Spline>(t_i, spline, spline::DerivativeOrder::Second)};
        measurements.push_back(C3Measurement{t_i, acceleration_i.value(), spline::DerivativeOrder::Second});
    }

    return {spline, measurements};
}

TEST_F(OptimizationSplineNonlinearRefinementFixture, TestNoisyR3SplineNonlinearRefinement) {
    auto const simulated_measurements{CalculateMeasurements<spline::R3Spline>()};

    // Make a copy and add noise so the optimization has to do some work :)
    spline::CubicBSplineC3 initialization{spline_};
    for (size_t i{0}; i < std::size(initialization.control_points); ++i) {
        initialization.control_points[i].array() += 0.1 * i;
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
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(spline_.control_points[i], 1e-6));
    }
}

TEST(OptimizationSo3SplineNonlinearRefinement, TestNoisySo3SplineNonlinearRefinement) {
    auto const [gt_spline, simulated_measurements]{R3SplineOptimizationTestData()};

    spline::CubicBSplineC3 initialization{gt_spline};
    for (size_t i{0}; i < std::size(initialization.control_points); ++i) {
        initialization.control_points[i].array() += 0.1 * i;
    }

    optimization::CubicBSplineC3Refinement handler{initialization};
    for (auto const& measurement : simulated_measurements) {
        bool const success{handler.AddConstraint<spline::So3Spline>(measurement)};
        EXPECT_TRUE(success);
    }

    bool const success{handler.AddConstraint<spline::So3Spline>({66666, {0, 0, 0}, spline::DerivativeOrder::Null})};
    EXPECT_FALSE(success);

    ceres::Solver::Summary const summary{handler.Solve()};
    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::CubicBSplineC3 const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < std::size(optimized_spline.control_points); ++i) {
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(gt_spline.control_points[i], 1e-1));
    }
}
