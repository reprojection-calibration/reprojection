#include "new_sphere_trajectory.hpp"

#include <gtest/gtest.h>

namespace reprojection::testing_mocks {

TEST(TestingMocksNewSphereTrajectory, TestTrajectoryPosition) {
    Vector3d const origin_w{0, 0, 0};
    double const radius{2};

    Vector3d result{TrajectoryPosition(0, origin_w, radius)};
    EXPECT_TRUE(result.isApprox(Vector3d{0, 2, 0}));

    // Given a speed_factor of 0.1 we expect that every five and ten seconds we find ourselves on the equator of the
    // circle again (i.e. z=0). The x and y values here are just a heuristic.
    result = TrajectoryPosition(5e9, origin_w, radius);
    EXPECT_TRUE(result.isApprox(Vector3d{0.61803398874989479, -1.9021130325903071, 0}));

    result = TrajectoryPosition(10e9, origin_w, radius);
    EXPECT_TRUE(result.isApprox(Vector3d{-1.1755705045849465, 1.6180339887498949, 0}));
}

}  // namespace reprojection::testing_mocks