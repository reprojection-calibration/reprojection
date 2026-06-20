#include "optimization/extrinsic_optimization.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "spline/spline_initialization.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;
namespace tu = testing_utilities;

TEST(OptimizationExtrinsicOptimization, TestExtrinsicOptimization) {
    double const duration_s{60};
    CameraInfo const camera_info{"cam", CameraModel::Pinhole, tu::image_bounds};

    auto const [targets, frames]{
        testing_mocks::GenerateMvgData(camera_info, CameraState{tu::pinhole_intrinsics}, duration_s, 10)};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(duration_s, 40)};

    Frames invert_frames;
    for (auto const& [timestamp_ns, frame_i] : frames) {
        invert_frames.insert({timestamp_ns, {geometry::Log(geometry::Exp(frame_i.pose).inverse())}});
    }

    spline::Se3Spline const initial_spline{spline::InitializeSe3SplineState(invert_frames, 100)};

    std::string const imu_name{"imu"};
    ImuCamExtrinsic const initial_extrinsic{{imu_name, camera_info.sensor_name, Vector6d::Zero()}, Vector3d::Zero()};

    auto const [optimized_spline, optimized_extrinsic]{optimization::ExtrinsicOptimization(
        imu_data, initial_spline, initial_extrinsic, camera_info, targets, {tu::pinhole_intrinsics})};

    std::cout << optimized_extrinsic.gravity.transpose() << std::endl;

    EXPECT_FALSE(true);
}

// See comments in TEST(OptimizationBundleAdjustment, TestEvaluateReprojectionResiduals) for context.
TEST(OptimizationExtrinsicOptimization, TestReprojectionErrorSpline) {
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

    CameraInfo const sensor{"", CameraModel::Pinhole, tu::image_bounds};
    CameraMeasurements const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};
    auto const camera_state{CameraState{tu::pinhole_intrinsics}};

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

TEST(OptimizationExtrinsicOptimization, TestEvaluateImuError) {
    // TODO(Jack): Are we really sure that this test reflects the camera calibration case? In the camera calibration
    // case the trajectory is actually inversed (look at the mvg data generator). Lets try this on real data :)
    auto const [imu_data, spline_w_co]{testing_mocks::GenerateImuData(10, 20)};

    ImuCamExtrinsic const extrinsic{Extrinsic{"imu", "cam", Array6d::Zero()}, {0, 0, 9.81}};
    auto const errors{optimization::EvaluateImuError(imu_data, extrinsic, spline_w_co)};

    EXPECT_EQ(std::size(errors), 195);
    for (auto const& error : errors) {
        EXPECT_TRUE(error.second.delta_angular_velocity.isZero(1e-3));
        std::cout << error.second.delta_linear_acceleration.transpose() << std::endl;
        EXPECT_TRUE(error.second.delta_linear_acceleration.isApproxToConstant(0));
    }
}