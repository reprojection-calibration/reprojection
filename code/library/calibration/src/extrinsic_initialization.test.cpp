#include "extrinsic_initialization.hpp"

#include <gtest/gtest.h>

#include "spline/spline_initialization.hpp"
#include "testing_mocks/data_generators.hpp"

using namespace reprojection;

TEST(CalibrationCameraImuInitialization, TestEstimateGravity) {
    double const duration_s{10};
    auto const [imu_data, spline_w_b]{testing_mocks::GenerateImuData(duration_s, 20)};

    auto const so3_co_w{spline::CubicBSplineC3{spline_w_b.So3(), spline_w_b.GetTimeHandler()}};
    auto imu_linear_acceleration{calibration::ExtractLinearAcceleration(imu_data)};
    Matrix3d const R_imu_co{Matrix3d::Identity()};

    Vector3d const gravity_w{calibration::EstimateGravity(so3_co_w, imu_linear_acceleration, R_imu_co)};

    // TODO(Jack): It would be cool if the gravity was exactly [0, 0, g] but the data does not provide us that. Is there
    // a bug in the data generation? I know the ends of the spline might cause problems.
    Vector3d const gt_gravity_w{-0.00626548, 0.0330134, 9.80659};
    EXPECT_TRUE(gravity_w.isApprox(gt_gravity_w, 1e-4));
}
