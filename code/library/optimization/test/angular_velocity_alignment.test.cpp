#include "optimization/angular_velocity_alignment.hpp"

#include <gtest/gtest.h>

#include "testing_mocks/data_generators.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

TEST(OptimizationAngularVelocityAlignment, TestAngularVelocityAlignment) {
    auto const [imu_data, spline]{testing_mocks::GenerateImuData(60, 50)};

    // Rotate every IMU velocity by some arbitrary rotation matrix and then check that this is recovered by the
    // optimization.
    static Matrix3d const R{{0, -1, 0},  //
                            {0, 0, -1},
                            {1, 0, 0}};
    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        omega_imu.insert({timestamp_ns, {R * data_i.angular_velocity}});
    }

    auto const [aa_co_imu, diagnostics]{optimization::AngularVelocityAlignment(omega_imu, spline)};

    // TODO(Jack): I wanted both the aa_co_imu and final cost to be more exact! But I think there is some numerical
    // inconsistency in the test data generation that means we need the large tolerance (1e-3) and the non-zero cost
    // (0.002065...). Can we engineer the test data to make this more exact? Or is it some other problem?
    EXPECT_TRUE(aa_co_imu.matrix().isApprox(geometry::Log(R), 1e-3));
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_NEAR(diagnostics.solver_summary.final_cost, 0.0020650323538566667, 1e-6);
}