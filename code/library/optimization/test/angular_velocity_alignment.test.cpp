#include "optimization/angular_velocity_alignment.hpp"

#include <gtest/gtest.h>

#include "spline/spline_initialization.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

using namespace reprojection;

VelocityMeasurements ExtractAngularVelocity(ImuMeasurements const& imu_data) {
    VelocityMeasurements imu_angular_velocity;
    for (auto const& [timestamp_ns, data_i] : imu_data) {
        imu_angular_velocity.insert({timestamp_ns, {data_i.angular_velocity}});
    }

    return imu_angular_velocity;
}  // LCOV_EXCL_LINE

TEST(OptimizationAngularVelocityAlignment, TestAngularVelocityAlignment) {
    double const duration_s{40};
    auto [imu_data, _]{testing_mocks::GenerateImuData(duration_s, 20)};

    // TODO(Jack): Should we just make a version of the data generation functions that also returns the camera pose
    // spline? We have this in multiple places now
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [_1, camera_poses]{testing_mocks::GenerateMvgData(sensor, gt_intrinsics, duration_s, 10)};
    spline::Se3Spline const spline_co_w{spline::InitializeSe3SplineState(camera_poses, 50)};

    VelocityMeasurements const omega_imu{ExtractAngularVelocity(imu_data)};
    auto const [aa_co_imu, diagnostics]{optimization::AngularVelocityAlignment(omega_imu, spline_co_w)};

    std::cout << geometry::Exp<double>(aa_co_imu) << std::endl;
    std::cout << "break" << std::endl;
    std::cout << geometry::Exp<double>(aa_co_imu).inverse() << std::endl;

    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_NEAR(diagnostics.solver_summary.final_cost, 0.0028202505264716135, 1e-6);
}