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
    double const duration_s{10};
    CameraInfo const camera_info{"cam", CameraModel::Pinhole, tu::image_bounds};

    auto const [targets, poses_co_w]{
        testing_mocks::GenerateMvgData(camera_info, CameraState{tu::pinhole_intrinsics}, duration_s, 10)};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(duration_s, 20)};

    Frames poses_w_co;
    for (auto const& [timestamp_ns, pose_co_w] : poses_co_w) {
        poses_w_co.insert({timestamp_ns, {geometry::Log(geometry::Exp(pose_co_w.pose).inverse())}});
    }
    spline::Se3Spline const spline_w_co{spline::InitializeSe3SplineState(poses_w_co, 50)};

    // TODO(Jack): These are heuristic values from running the optimization. We start the optimization here so it runs
    // the test as fast as possible. Ideally the result would actually be the values that the test data was created with
    // but there is some error here. Instead of getting exactly gravity back or just camera rotation matrix back with
    // zero translation we get roughly these values back with some errors in all directions. This might be because there
    // or is a problem with the test data creation or it might be the nature of the problem itself, for example how it
    // optimizes the spline to have minimum energy. Constraints like that might introduce errors elsewhere as we do not
    // handle the relative weighting between these things in any intelligent or principled manner. For now these test
    // values are close enough and serve as the canary in the coal mine :)
    ImuCamExtrinsic const initial_extrinsic{
        {"imu", camera_info.sensor_name, Vector6d{-1.19516, 1.17219, -1.23556, -0.0242935, 0.0530558, 0.0251949}},
        Vector3d{-0.212548, -0.293729, 9.79995}};

    auto const [_1, optimized_extrinsic]{optimization::ExtrinsicOptimization(
        imu_data, spline_w_co, initial_extrinsic, camera_info, targets, {tu::pinhole_intrinsics})};

    EXPECT_TRUE(optimized_extrinsic.tf.se3_a_b.isApprox(initial_extrinsic.tf.se3_a_b, 1e-2));
    EXPECT_TRUE(optimized_extrinsic.gravity.isApprox(initial_extrinsic.gravity, 1e-2));
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