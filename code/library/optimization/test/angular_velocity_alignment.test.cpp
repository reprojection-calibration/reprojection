#include "optimization/angular_velocity_alignment.hpp"

#include <gtest/gtest.h>

#include "spline/spline_initialization.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

// COPY AND PASTED
VelocityMeasurements ExtractAngularVelocity(ImuMeasurements const& imu_data) {
    VelocityMeasurements imu_angular_velocity;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        imu_angular_velocity.insert({timestamp_ns, {data_i.angular_velocity}});
    }

    return imu_angular_velocity;
}  // LCOV_EXCL_LINE

TEST(OptimizationAngularVelocityAlignment, TestAngularVelocityAlignment) {
    double const duration_s{30};
    auto [imu_data, spline_w_b]{testing_mocks::GenerateImuData(duration_s, 20)};

    VelocityMeasurements const omega_imu{ExtractAngularVelocity(imu_data)};
    auto const [aa_imu_co, diagnostics]{optimization::AngularVelocityAlignment(omega_imu, spline_w_b)};

    EXPECT_TRUE(aa_imu_co.isZero(1e-3)); // Identity matrix
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_NEAR(diagnostics.solver_summary.final_cost, 0, 1e-6);
}