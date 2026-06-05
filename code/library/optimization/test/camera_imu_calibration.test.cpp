#include "optimization/camera_imu_calibration.hpp"

#include <gtest/gtest.h>

#include "spline/spline_initialization.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(OptimizationCameraImuCalibration, TestExtrinsicOptimization) {
    CameraInfo const camera_info{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    uint64_t const timespan_ns{1'000'000'000};

    auto const [targets, poses]{testing_mocks::GenerateMvgData(camera_info, gt_intrinsics, 50, timespan_ns, false)};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(200, timespan_ns)};

    spline::Se3Spline const initial_spline{spline::InitializeSe3SplineState(poses, 500)};
    Array6d const initial_tf_imu_co{Array6d::Zero()};
    Array3d const initial_gravity_w{Array3d::Zero()};

    auto const [spline, tf_imu_co, gravity_w, diagnostics]{optimization::ExtrinsicOptimization(
        imu_data, initial_spline, initial_tf_imu_co, initial_gravity_w, camera_info, targets, gt_intrinsics)};
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);
}

// See comments in TEST(OptimizationBundleAdjustment, TestEvaluateReprojectionResiduals) for context.
TEST(OptimizationCameraImuCalibration, TestReprojectionErrorSpline) {
    MatrixX2d const gt_pixels{{-1, -1},  //
                              {350, 230},
                              {-1, -1},
                              {-1, -1},
                              {365, 245}};
    MatrixX3d const gt_points{{0, 0, -600},  //
                              {0, 0, 600},
                              {0, 0, -600},
                              {0, 0, -600},
                              {0, 0, 600}};
    ArrayX2d const gt_residuals{{256, 256},  //
                                {-10, -10},
                                {256, 256},
                                {256, 256},
                                {5, 5}};

    uint64_t const timestamp_ns{0};

    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraMeasurements const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};
    auto const camera_state{CameraState{testing_utilities::pinhole_intrinsics}};

    // The control points for a spline with one segment that is simply the constant identity transform.
    spline::Matrix2NK<double> control_points;
    control_points << Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero();
    spline::Se3Spline const spline{control_points, {0, 1}};

    auto const [poses, errors]{optimization::ReprojectionErrorSpline(spline, sensor, targets, camera_state)};
    EXPECT_EQ(std::size(poses), 1);
    EXPECT_TRUE(poses.at(timestamp_ns).pose.isApproxToConstant(0));
    EXPECT_EQ(std::size(errors), 1);
    EXPECT_TRUE(errors.at(timestamp_ns).isApprox(gt_residuals))
        << "Result:\n"
        << errors.at(timestamp_ns).transpose() << "\nexpected result:\n"
        << gt_residuals.transpose();
}

TEST(OptimizationCameraImuCalibration, TestEvaluateImuError) {
    // TODO(Jack): Are we really sure that this test reflects the camera calibration case? In the camera calibration
    // case the trajectory is actually inversed (look at the mvg data generator). Lets try this on real data :)
    auto [imu_data, trajectory]{testing_mocks::GenerateImuData(100, 1'000'000'000)};

    Array6d const tf_imu_co{Array6d::Zero()};
    Array3d const gravity_w{Array3d::Zero()};
    auto const errors{optimization::EvaluateImuError(imu_data, trajectory, tf_imu_co, gravity_w)};

    EXPECT_EQ(std::size(errors), 100);
    for (auto const& error : errors) {
        EXPECT_TRUE(error.second.delta_angular_velocity.isApproxToConstant(0));
        EXPECT_TRUE(error.second.delta_linear_acceleration.isApproxToConstant(0));
    }
}