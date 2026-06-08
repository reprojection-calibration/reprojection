#include <gtest/gtest.h>

#include "optimization/extrinsic_optimization.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

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

    auto const [poses, errors]{optimization::ReprojectionErrorSpline(sensor, targets, camera_state, spline)};
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

    ImuCamExtrinsic const extrinsic{Extrinsic{"imu", "cam", Array6d::Zero()}, Array3d::Zero()};
    auto const errors{optimization::EvaluateImuError(imu_data, extrinsic, trajectory)};

    EXPECT_EQ(std::size(errors), 100);
    for (auto const& error : errors) {
        EXPECT_TRUE(error.second.delta_angular_velocity.isApproxToConstant(0));
        EXPECT_TRUE(error.second.delta_linear_acceleration.isApproxToConstant(0));
    }
}