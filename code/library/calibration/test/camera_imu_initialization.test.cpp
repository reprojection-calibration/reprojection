#include "calibration/camera_imu_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// TODO(Jack): Add gravity to the IMU test mock data and rotate it to another frame so we can actually stress the
//  underlying functions. This test is currently pretty weak because the input test mock data is so simple/does not
//  fully reflect reality where the camera and IMU are not in the exact same frame.

TEST(CalibrationCameraImuExtrinsicInitialization, TestCameraImuExtrinsicInitialization) {
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    uint64_t const timespan_ns{10000000000};
    auto const [targets, camera_frames]{
        testing_mocks::GenerateMvgData(sensor, CameraState{testing_utilities::pinhole_intrinsics}, 200, timespan_ns)};
    ImuMeasurements const imu_data{testing_mocks::GenerateImuData(1000, timespan_ns)};

    auto const [rotation_result, gravity]{calibration::EstimateCameraImuRotationAndGravity(camera_frames, imu_data)};
    auto const [R_co_imu, diagnostics]{rotation_result};

    EXPECT_TRUE(R_co_imu.isApprox(Matrix3d::Identity()));
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::CONVERGENCE);
    EXPECT_TRUE(gravity.isApprox(Vector3d::Zero()));
}
