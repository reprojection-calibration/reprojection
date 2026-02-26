#include "optimization/camera_imu_nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// See comments in TEST(OptimizationCameraNonlinearRefinement, TestEvaluateReprojectionResiduals) for context.
TEST(OptimizationCameraImuNonlinearRefinement, TestEvaluateSplineReprojectionResiduals) {
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

    ReprojectionErrors const residuals{
        optimization::SplineReprojectionResiduals(sensor, targets, camera_state, spline)};
    EXPECT_EQ(std::size(residuals), 1);
    EXPECT_TRUE(residuals.at(timestamp_ns).isApprox(gt_residuals))
        << "Result:\n"
        << residuals.at(timestamp_ns).transpose() << "\nexpected result:\n"
        << gt_residuals.transpose();
}