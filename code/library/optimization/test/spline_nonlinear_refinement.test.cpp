#include "optimization/spline_nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_evaluation_concept.hpp"
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
        requires spline::CanEvaluateCubicBSplineC3<T_Model>
    std::vector<spline::C3Measurement> CalculateMeasurements() {
        using namespace spline;
        using C3Measurement = spline::C3Measurement;

        std::vector<C3Measurement> measurements;
        // Given five control points we get three valid time segments of length delta_t_ns_
        for (size_t i{0}; i < 3 * delta_t_ns_; ++i) {
            std::uint64_t const t_i{t0_ns_ + i};

            auto const position_i{EvaluateSpline<T_Model>(t_i, spline_, DerivativeOrder::Null)};
            measurements.push_back(C3Measurement{t_i, position_i.value(), DerivativeOrder::Null});

            auto const velocity_i{EvaluateSpline<T_Model>(t_i, spline_, DerivativeOrder::First)};
            measurements.push_back(C3Measurement{t_i, velocity_i.value(), DerivativeOrder::First});

            auto const acceleration_i{EvaluateSpline<T_Model>(t_i, spline_, DerivativeOrder::Second)};
            measurements.push_back(C3Measurement{t_i, acceleration_i.value(), DerivativeOrder::Second});
        }

        return measurements;
    }

    std::uint64_t t0_ns_{100};
    std::uint64_t delta_t_ns_{50};
    spline::CubicBSplineC3 spline_{t0_ns_, delta_t_ns_};

   private:
    static double Squared(double const x) { return x * x; }
};

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

TEST_F(OptimizationSplineNonlinearRefinementFixture, TestNoisySo3SplineNonlinearRefinement) {
    auto const simulated_measurements{CalculateMeasurements<spline::So3Spline>()};

    spline::CubicBSplineC3 initialization{spline_};
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
        EXPECT_TRUE(optimized_spline.control_points[i].isApprox(spline_.control_points[i], 1e-1));
    }
}
