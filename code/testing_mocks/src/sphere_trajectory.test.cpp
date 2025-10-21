#include "sphere_trajectory.hpp"

#include <gtest/gtest.h>

using namespace reprojection::testing_mocks;

// WARN(Jack): In this test we do not test that the direction the tfs are pointing at is correct! Only the positions of
// them.
TEST(TestingMocksSphereTrajectory, TestSphereTrajectory) {
    double const sphere_radius{1.5};
    Eigen::Vector3d const sphere_origin{1, 2, 3};
    CameraTrajectory const config{{0, 0, 0}, sphere_radius, sphere_origin};

    std::vector<Eigen::Isometry3d> const tfs{SphereTrajectory(config)};

    double radius{0};
    Eigen::Vector3d centroid{0, 0, 0};
    for (auto const& tf : tfs) {
        radius += (tf.translation() - sphere_origin).norm();
        centroid += tf.translation();
    }
    radius = radius / std::size(tfs);
    centroid = centroid / std::size(tfs);

    EXPECT_FLOAT_EQ(radius, sphere_radius);
    EXPECT_TRUE(centroid.isApprox(sphere_origin));
}

TEST(TestingMocksSphereTrajectory, TestTrackPoint) {
    Eigen::Vector3d const world_origin{0, 0, 0};

    Eigen::Vector3d const null{TrackPoint(world_origin, world_origin)};
    EXPECT_TRUE(null.isApprox(Eigen::Vector3d{0, 0, 0}));

    Eigen::Vector3d const camera_position_x{1, 0, 0};
    Eigen::Vector3d const x{TrackPoint(world_origin, camera_position_x)};
    EXPECT_TRUE(x.isApprox(Eigen::Vector3d{0, -M_PI / 2, 0}));

    Eigen::Vector3d const camera_position_y{0, 1, 0};
    Eigen::Vector3d const y{TrackPoint(world_origin, camera_position_y)};
    EXPECT_TRUE(y.isApprox(Eigen::Vector3d{M_PI / 2, 0, 0}));

    // NOTE(Jack): The following two cases are the cases where the origin and camera position are in the same line
    // defined by the xy values. For example, the case when (0,0,z) for both the origin and camera position, this is a
    // degenerate case. The logic in TrackPoint() essentially detects this edge case and returns either pi or 0 (because
    // they are already aligned along the z direction), depending on if the camera is above or below the world origin.

    // Need to do a 180 to face directly downwards.
    Eigen::Vector3d const camera_position_plus_z{0, 0, 1};
    Eigen::Vector3d const plus_z{TrackPoint(world_origin, camera_position_plus_z)};
    EXPECT_TRUE(plus_z.isApprox(Eigen::Vector3d{M_PI, 0, 0}));

    // Already facing directly upwards towards the world origin, no rotation needed.
    Eigen::Vector3d const camera_position_minus_z{0, 0, -1};
    Eigen::Vector3d const minus_z{TrackPoint(world_origin, camera_position_minus_z)};
    EXPECT_TRUE(minus_z.isApprox(Eigen::Vector3d{0, 0, 0}));
}