#include "camera_imu_initialization.hpp"

#include <gtest/gtest.h>

#include "spline/spline_initialization.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(CalibrationCameraImuInitialization, TestEstimateGravity) {
    double const duration_s{10};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(duration_s, 20)};

    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [_1, camera_poses]{testing_mocks::GenerateMvgData(sensor, gt_intrinsics, duration_s, 10)};

    spline::Se3Spline const spline_co_w{spline::InitializeSe3SplineState(camera_poses, 50)};

    auto const so3_co_w{spline::CubicBSplineC3{spline_co_w.So3(), spline_co_w.GetTimeHandler()}};
    auto imu_linear_acceleration{calibration::ExtractLinearAcceleration(imu_data)};
    static Matrix3d const R_co_imu{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    static Matrix3d const R_imu_co{R_co_imu.inverse()};

    Vector3d const gravity_w{calibration::EstimateGravity(so3_co_w, imu_linear_acceleration, R_imu_co)};

    Vector3d const gt_gravity_w{0.066929787601231625, -0.058423503100995429, 9.8062475657265065};
    EXPECT_TRUE(gravity_w.isApprox(gt_gravity_w));
}
