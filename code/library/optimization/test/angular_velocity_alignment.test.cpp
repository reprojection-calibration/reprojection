#include "optimization/angular_velocity_alignment.hpp"

#include <gtest/gtest.h>

#include "testing_mocks/imu_data_generator.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

TEST(OptimizationAngularVelocityAlignment, TestAngularVelocityAlignment) {
    auto const [imu_data, trajectory]{testing_mocks::GenerateImuData(100, 1'000'000'000)};

    // Rotate every IMU velocity by some arbitrary rotation matrix and then check that this is recovered by the
    // optimization.
    static Matrix3d const R{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    VelocityMeasurements omega_imu;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        omega_imu.insert({timestamp_ns, {R * data_i.angular_velocity}});
    }

    auto const [aa_co_imu, diagnostics]{
        optimization::AngularVelocityAlignment(omega_imu, trajectory)};

    EXPECT_TRUE(aa_co_imu.matrix().isApprox(geometry::Log(R)));
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_NEAR(diagnostics.solver_summary.final_cost, 0, 1e-18);
}