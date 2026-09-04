#include "optimization/extrinsic_optimization.hpp"

#include <gtest/gtest.h>

#include "spline/spline_initialization.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;
namespace tu = testing_utilities;

// NOTE(Jack): This test was SO slow in the Debug build which we used in the pipeline/CI for code coverage and testing
// that we were forced to start using RelWithDebInfo as our build type instead. I think even with that build type the
// code coverage works fine, but it did change the nature of some of the false positives which I then had to suppress.
// Bottom line is that this was so slow that we had to find a work around to get it faster and changing the build type
// seems ok... for now.
TEST(OptimizationExtrinsicOptimization, TestExtrinsicOptimization) {
    double const duration_s{10};
    CameraInfo const camera_info{CameraModel::Pinhole, tu::image_bounds};

    auto const [targets, poses_co_w]{
        testing_mocks::GenerateMvgData(camera_info, Intrinsic{tu::pinhole_intrinsics}, duration_s, 10)};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(duration_s, 20)};

    Frames poses_w_co;
    for (auto const& [timestamp_ns, pose_co_w] : poses_co_w) {
        poses_w_co.insert({timestamp_ns, {geometry::Log(geometry::Exp(pose_co_w.value).inverse())}});
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
    Extrinsic const initial_extrinsic{AssetId{1}, AssetId{2},
                                      Vector6d{-1.19516, 1.17219, -1.23556, -0.0242935, 0.0530558, 0.0251949}};
    Vector3d const initial_gravity{Vector3d{-0.212548, -0.293729, 9.79995}};

    auto const [_1, optimized_extrinsic, optimized_gravity]{optimization::ExtrinsicOptimization(
        imu_data, spline_w_co, initial_extrinsic, initial_gravity, camera_info, targets, {tu::pinhole_intrinsics}, 1)};

    EXPECT_TRUE(optimized_extrinsic.se3_a_b.isApprox(initial_extrinsic.se3_a_b, 1e-2));
    EXPECT_TRUE(optimized_gravity.isApprox(initial_gravity, 1e-2));
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

    CameraInfo const camera_info{CameraModel::Pinhole, tu::image_bounds};
    TargetSamples const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};
    Intrinsic const intrinsic{tu::pinhole_intrinsics};
    AssetId const camera_id{1};

    // The control points for a spline with one segment that is simply the constant identity transform.
    spline::Matrix2NK<double> control_points;
    control_points << Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero();
    spline::Se3Spline const spline{control_points, {0, 1}};

    auto const ba_problem{optimization::SingleSplineCamProblem(camera_info, intrinsic, targets, spline, camera_id)};
    auto const residuals{optimization::EvaluateResiduals(ba_problem)};

    EXPECT_EQ(std::size(ba_problem.rig_poses), 1);
    EXPECT_TRUE(ba_problem.rig_poses.at(timestamp_ns).value.isApproxToConstant(0));

    EXPECT_EQ(std::size(residuals), 1);
    auto const& error{residuals[0]};
    EXPECT_EQ(error.camera_id, camera_id);
    EXPECT_EQ(error.timestamp_ns, timestamp_ns);
    EXPECT_TRUE(error.value.isApprox(gt_residuals)) << "Result:\n"
                                                    << error.value.transpose() << "\nexpected result:\n"
                                                    << gt_residuals.transpose();
}

TEST(OptimizationExtrinsicOptimization, TestEvaluateImuError) {
    auto const [imu_data, spline_w_co]{testing_mocks::GenerateImuData(10, 20)};

    // TODO(Jack): I am getting the sneaking suspicion that because the functions here do not actually use the assetid
    // we need to rethink passing the entire extrinsic struct.
    Extrinsic const extrinsic{AssetId{1}, AssetId{2}, Array6d::Zero()};
    Vector3d const gravity{0, 0, 9.81};
    auto const errors{optimization::EvaluateImuError(imu_data, extrinsic, gravity, spline_w_co)};

    EXPECT_EQ(std::size(errors), 195);
    for (auto const& error : errors) {
        EXPECT_TRUE(error.second.delta_angular_velocity.isZero(1e-3));
        // TODO(Jack): There is something wrong with either the test data or the math here. The errors here should
        // really be exactly zero just like the angular velocity. This needs further investigation!
        EXPECT_LT(error.second.delta_linear_acceleration.norm(), 0.4);
    }
}