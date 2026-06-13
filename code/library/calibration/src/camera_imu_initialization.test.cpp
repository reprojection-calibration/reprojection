#include "camera_imu_initialization.hpp"

#include <gtest/gtest.h>

#include "testing_mocks/data_generators.hpp"

using namespace reprojection;

// Vector3d EstimateGravity(CubicBSplineC3 const& camera_orientation, AccelerationMeasurements const& imu_acceleration,
// Matrix3d const& R_imu_co)

TEST(CalibrationCameraImuInitialization, TestEstimateGravity) {
    auto const [imu_data, spline]{testing_mocks::GenerateImuData(60, 10)};

    auto const orientation_spline{spline::CubicBSplineC3{spline.So3(), spline.GetTimeHandler()}};
    auto imu_linear_acceleration{calibration::ExtractLinearAcceleration(imu_data)};
    Matrix3d const R_imu_co{Matrix3d::Identity()};


    Vector3d const gravity_w{calibration::EstimateGravity(orientation_spline, imu_linear_acceleration, R_imu_co)};

    std::cout << gravity_w << std::endl;

    EXPECT_EQ(1, 2);
}
