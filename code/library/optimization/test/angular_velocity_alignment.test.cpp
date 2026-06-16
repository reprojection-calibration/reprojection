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
    // WARN(Jack): This duration might need to be longer to get a real result. But for now we keep it short because this
    // is a heuristic anyway and the test is a basically a placeholder until we figure out what's wrong.
    double const duration_s{10};
    auto [imu_data, _]{testing_mocks::GenerateImuData(duration_s, 20)};

    // TODO(Jack): Should we just make a version of the data generation functions that also returns the camera pose
    // spline? We have this in multiple places now
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [_1, camera_poses]{testing_mocks::GenerateMvgData(sensor, gt_intrinsics, duration_s, 10)};
    spline::Se3Spline const spline_co_w{spline::InitializeSe3SplineState(camera_poses, 50)};

    VelocityMeasurements const omega_imu{ExtractAngularVelocity(imu_data)};
    auto const [aa_imu_co, diagnostics]{optimization::AngularVelocityAlignment(omega_imu, spline_co_w)};

    // WARN(Jack): This test is not satisfactory at all! I want these tests to have exactly cost and return the exact
    // result (i.e. the canonical camera-body rotation matrix). But there is something going on and this is not
    // happening. This is going to be debugged, and hopefully we are removing this comment soon.

    // Heuristics!
    Array3d const gt_aa_imu_co{1.01824, -2.72506, -0.520739};
    EXPECT_TRUE(aa_imu_co.isApprox(gt_aa_imu_co, 1e-2));
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_NEAR(diagnostics.solver_summary.final_cost, 2.1115451244140173, 1e-6);
}