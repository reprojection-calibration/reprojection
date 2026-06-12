#include "../include/testing_mocks/new_sphere_trajectory.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(TestingMocksNewSphereTrajectory, TestTrajectoryPosition) {
    Vector3d const origin_w{0, 0, 0};
    double const radius{2};

    Vector3d result{testing_mocks::TrajectoryPosition(0, origin_w, radius)};
    EXPECT_TRUE(result.isApprox(Vector3d{0, 2, 0}));

    // Given a speed_factor of 0.1 we expect that every five and ten seconds we find ourselves on the equator of the
    // circle again (i.e. z=0). The x and y values here are just a heuristic.
    result = testing_mocks::TrajectoryPosition(5e9, origin_w, radius);
    EXPECT_TRUE(result.isApprox(Vector3d{0.61803398874989479, -1.9021130325903071, 0}));

    result = testing_mocks::TrajectoryPosition(10e9, origin_w, radius);
    EXPECT_TRUE(result.isApprox(Vector3d{-1.1755705045849465, 1.6180339887498949, 0}));
}

TEST(TestingMocksNewSphereTrajectory, TestLookAtRotationWorldBody) {
    Vector3d const target_w{0, 0, 0};

    Vector3d position_w{0, 0, 0};
    Matrix3d R_w_b{testing_mocks::LookAtRotationWorldBody(position_w, target_w, std::nullopt)};
    EXPECT_TRUE(R_w_b.isApprox(Matrix3d::Identity()));

    // TOOD(Jack): Are there any other cases we can/should test here?
}

TEST(TestingMocksNewSphereTrajectory, TestSampleTimes) {
    auto const times_ns{testing_mocks::SampleTimes(2, 100)};
    EXPECT_EQ(times_ns.count(), 199);
    EXPECT_EQ(times_ns[0], 0);
    EXPECT_EQ(times_ns[199], 1999999949);
}
