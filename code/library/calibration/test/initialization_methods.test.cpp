#include "calibration/initialization_methods.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(CalibrationInitializationMethods, TestInitializeIntrinsics) {
    // TODO(Jack): Use a fixture!!!
    CameraInfo const sensor{"", CameraModel::DoubleSphere, testing_utilities::image_bounds};
    CameraState const intrinsics{testing_utilities::double_sphere_intrinsics};
    auto const [targets, _]{testing_mocks::GenerateMvgData(sensor, intrinsics, 10, 1)};

    auto const result{
        calibration::InitializeIntrinsics(sensor.camera_model, sensor.bounds.v_max, sensor.bounds.u_max, targets)};

    ASSERT_TRUE(result.has_value());
}

TEST(CalibrationInitializationMethods, TestPoseInitialization) {
    // Setup test data
    CameraInfo const camera_info{"", CameraModel::DoubleSphere, testing_utilities::image_bounds};
    CameraState const intrinsics{testing_utilities::double_sphere_intrinsics};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(camera_info, intrinsics, 60, 1)};

    // Act
    Frames const linear_solution{calibration::PoseInitialization(camera_info, targets, intrinsics)};

    // Assert
    EXPECT_EQ(std::size(linear_solution), 56);
    for (auto const& [timestamp_ns, frame_i] : linear_solution) {
        Array6d const gt_aa_co_w{gt_frames.at(timestamp_ns).pose};
        EXPECT_TRUE(frame_i.pose.isApprox(gt_aa_co_w, 1e-12)) << "Linear pose initialization result:\n"
                                                              << frame_i.pose.transpose() << "\nGround truth:\n"
                                                              << gt_aa_co_w.transpose();
    }
}

TEST(CalibrationInitializationMethods, TestEstimateCameraImuAlignment) {
    auto [imu_data, spline]{testing_mocks::GenerateImuData(10, 50)};

    auto const [rotation_result, gravity_w]{calibration::EstimateCameraImuAlignment(spline, imu_data)};
    auto const [aa_imu_co, diagnostics]{rotation_result};

    // Heuristic! I wish it was really exactly the identity matrix, but it's a little off.
    EXPECT_TRUE(geometry::Exp<double>(aa_imu_co).isApprox(Matrix3d::Identity(), 1e-3));

    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_EQ(gravity_w.norm(), 9.8066500000000012);
    // TODO(Jack): I would expect the gravity to be all along the z-axis (or one single axis at least). I think this is
    // a sign we have something wrong here. It could be with the data generation itself or the initialization algorithm.
    Vector3d const heuristic_gravity_w{-0.21810377227268246, -3.9267490800174922, 8.9835102621192693};
    EXPECT_TRUE(gravity_w.isApprox(heuristic_gravity_w));
}