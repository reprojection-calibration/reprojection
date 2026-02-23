#include "optimization/angular_velocity_alignment.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

TEST(OptimizationAngularVelocityAlignment, TestAngularVelocityAlignment) {
    Matrix3d const gt_R_co_imu{{-1, 0, 0},  //
                               {0, 0, -1},
                               {0, -1, 0}};

    VelocityMeasurements omega_co;
    VelocityMeasurements omega_imu;
    for (uint64_t t{0}; t < 100; ++t) {
        Vector3d const omega_imu_i{Vector3d::Random()};
        Vector3d const omega_co_i{gt_R_co_imu * omega_imu_i};

        omega_co.insert({t, {omega_co_i}});
        omega_imu.insert({t, {omega_imu_i}});
    }

    auto const [R_co_imu, diagnostics]{optimization::AngularVelocityAlignment(omega_co, omega_imu)};
    EXPECT_TRUE(R_co_imu.isApprox(gt_R_co_imu, 1e-9)) << "Result:\n"
                                                      << R_co_imu << "\nexpected result:\n"
                                                      << gt_R_co_imu;
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_NEAR(diagnostics.solver_summary.final_cost, 0, 1e-18);
}