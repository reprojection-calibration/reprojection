#include "trajectory.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"

using namespace reprojection;

TEST(TestingMocksTrajectory, TestTrajectory) {
    auto const [poses, imu_data]{testing_mocks::Trajectory(60, 1, {2, 2, 2}, {0, 0, 0}, 1)};

    EXPECT_EQ(std::size(poses), 56);
    EXPECT_EQ(std::size(imu_data), 56);

    // Heuristic checks of both the pose and imu_data (first frame only).

    Frame const frame_x{*std::cbegin(poses)};
    EXPECT_EQ(frame_x.first, 2033898304);
    EXPECT_TRUE(frame_x.second.pose.isApprox(Array6d{0.824561, 0.480256, -2.12411, 2.02368, 2.29411, 2.95548}, 1e-4));

    ImuMeasurement const imu_data_x{*std::cbegin(imu_data)};
    EXPECT_EQ(imu_data_x.first, 2033898304);
    EXPECT_TRUE(imu_data_x.second.angular_velocity.isApprox(Vector3d{0.00175316, 0.100484, -0.101192}, 1e-4));
    EXPECT_TRUE(imu_data_x.second.linear_acceleration.isApprox(Vector3d{-6.5348, -0.446775, 6.78769}, 1e-4));
}

TEST(TestingMocksTrajectory, TestPositionWorld) {
    Vector3d const origin_w{0, 0, 0};
    double const radius{2};

    Vector3d result{testing_mocks::PositionWorld(0, origin_w, radius)};
    EXPECT_TRUE(result.isApprox(Vector3d{0, 2, 0}));

    // Given a speed_factor of 0.1 we expect that every five and ten seconds we find ourselves on the equator of the
    // circle again (i.e. z=0). The x and y values here are just a heuristic.
    result = testing_mocks::PositionWorld(5e9, origin_w, radius);
    EXPECT_TRUE(result.isApprox(Vector3d{0.61803398874989479, -1.9021130325903071, 0}));

    result = testing_mocks::PositionWorld(10e9, origin_w, radius);
    EXPECT_TRUE(result.isApprox(Vector3d{-1.1755705045849465, 1.6180339887498949, 0}));
}

TEST(TestingMocksTrajectory, TestLookAtRotationWorldBody) {
    Vector3d const target_w{0, 0, 0};

    Vector3d position_w{0, 0, 0};
    Matrix3d R_w_b{testing_mocks::LookAtRotationBodyToWorld(position_w, target_w, std::nullopt)};
    EXPECT_TRUE(R_w_b.isApprox(Matrix3d::Identity()));

    // TOOD(Jack): Are there any other cases we can/should test here?
}

TEST(TestingMocksTrajectory, TestSampleTimes) {
    auto const times_ns{testing_mocks::SampleTimes(2, 100)};
    EXPECT_EQ(times_ns.count(), 199);
    EXPECT_EQ(times_ns[0], 0);
    EXPECT_EQ(times_ns[199], 1999999949);
}
