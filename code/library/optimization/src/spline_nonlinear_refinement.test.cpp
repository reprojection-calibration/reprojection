#include "spline_nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_evaluation_concept.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

// TODO(Jack): In this file we see clearly the logical confusion of repeating the same logic three times for each type
//  of measurement. However it might be that in the real optimization we only have two types, the rotation velocity and
//  linear acceleration, therefore maybe we can actually remove this logic one day when we have tests that better
//  reflect reality of what the IMU actually measures and how the se3 spline actually works.

class OptimizationSplineNonlinearRefinementFixture : public ::testing::Test {
   protected:
    OptimizationSplineNonlinearRefinementFixture() {
        std::vector<double> const x{-2, -1, -0.5, 0.5, 1, 2};

        spline::MatrixNXd control_points{3, std::size(x)};
        for (size_t i{0}; i < std::size(x); ++i) {
            double const x_i{x[i]};
            control_points.col(i) = Vector3d{x_i, Squared(x_i), Squared(x_i)};
        }

        spline_ = spline::CubicBSplineC3(control_points, spline::TimeHandler{t0_ns_, delta_t_ns_});
    }

    template <typename T_Model>
        requires spline::CanEvaluateCubicBSplineC3<T_Model>
    std::tuple<PositionMeasurements, VelocityMeasurements, AccelerationMeasurements> CalculateMeasurements() {
        PositionMeasurements positions;
        VelocityMeasurements velocities;
        AccelerationMeasurements accelerations;

        // Given five control points we get three valid time segments of length delta_t_ns_
        for (size_t i{0}; i < 3 * delta_t_ns_; ++i) {
            std::uint64_t const timestamp_ns{t0_ns_ + i};

            auto const position_i{EvaluateSpline<T_Model>(spline_, timestamp_ns, spline::DerivativeOrder::Null)};
            positions.insert({timestamp_ns, {position_i.value()}});

            auto const velocity_i{EvaluateSpline<T_Model>(spline_, timestamp_ns, spline::DerivativeOrder::First)};
            velocities.insert({timestamp_ns, {velocity_i.value()}});

            auto const acceleration_i{EvaluateSpline<T_Model>(spline_, timestamp_ns, spline::DerivativeOrder::Second)};
            accelerations.insert({timestamp_ns, {acceleration_i.value()}});
        }

        return {positions, velocities, accelerations};
    }

    std::uint64_t t0_ns_{100};
    std::uint64_t delta_t_ns_{50};
    spline::CubicBSplineC3 spline_;

   private:
    static double Squared(double const x) { return x * x; }
};

using R3Spline = spline::R3Spline;
using So3Spline = spline::So3Spline;
using DerivativeOrder = spline::DerivativeOrder;

TEST_F(OptimizationSplineNonlinearRefinementFixture, TestNoisyR3SplineNonlinearRefinement) {
    auto const [positions, velocities, accelerations]{CalculateMeasurements<R3Spline>()};

    // Make a copy and add noise so the optimization has to do some work :)
    spline::CubicBSplineC3 initialization{spline_};
    for (size_t i{0}; i < initialization.Size(); ++i) {
        initialization.MutableControlPoints().col(i).array() += 0.1 * i;
    }

    optimization::CubicBSplineC3Refinement handler{initialization};
    for (auto const& [timestamp_ns, data] : positions) {
        // All simulated measurements come from valid time range
        EXPECT_TRUE(handler.AddConstraint<R3Spline>(timestamp_ns, data.position, DerivativeOrder::Null));
    }
    for (auto const& [timestamp_ns, data] : velocities) {
        EXPECT_TRUE(handler.AddConstraint<R3Spline>(timestamp_ns, data.velocity, DerivativeOrder::First));
    }
    for (auto const& [timestamp_ns, data] : accelerations) {
        EXPECT_TRUE(handler.AddConstraint<R3Spline>(timestamp_ns, data.acceleration, DerivativeOrder::Second));
    }

    // Check that we reject a bad measurement point outside the spline domain (bad time)
    EXPECT_FALSE(handler.AddConstraint<R3Spline>(66666, {0, 0, 0}, DerivativeOrder::Null));

    ceres::Solver::Summary const summary{handler.Solve()};
    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::CubicBSplineC3 const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < optimized_spline.Size(); ++i) {
        EXPECT_TRUE(optimized_spline.ControlPoints().col(i).isApprox(spline_.ControlPoints().col(i), 1e-6));
    }
}

TEST_F(OptimizationSplineNonlinearRefinementFixture, TestNoisySo3SplineNonlinearRefinement) {
    auto const [positions, velocities, accelerations]{CalculateMeasurements<So3Spline>()};

    spline::CubicBSplineC3 initialization{spline_};
    for (size_t i{0}; i < initialization.Size(); ++i) {
        initialization.MutableControlPoints().col(i).array() += 0.1 * i;
    }

    optimization::CubicBSplineC3Refinement handler{initialization};
    for (auto const& [timestamp_ns, data] : positions) {
        // All simulated measurements come from valid time range
        EXPECT_TRUE(handler.AddConstraint<So3Spline>(timestamp_ns, data.position, DerivativeOrder::Null));
    }
    for (auto const& [timestamp_ns, data] : velocities) {
        EXPECT_TRUE(handler.AddConstraint<So3Spline>(timestamp_ns, data.velocity, DerivativeOrder::First));
    }
    for (auto const& [timestamp_ns, data] : accelerations) {
        EXPECT_TRUE(handler.AddConstraint<So3Spline>(timestamp_ns, data.acceleration, DerivativeOrder::Second));
    }

    EXPECT_FALSE(handler.AddConstraint<So3Spline>(66666, {0, 0, 0}, DerivativeOrder::Null));

    ceres::Solver::Summary const summary{handler.Solve()};
    ASSERT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

    spline::CubicBSplineC3 const optimized_spline{handler.GetSpline()};
    for (size_t i{0}; i < optimized_spline.Size(); ++i) {
        EXPECT_TRUE(optimized_spline.ControlPoints().col(i).isApprox(spline_.ControlPoints().col(i), 1e-1));
    }
}
