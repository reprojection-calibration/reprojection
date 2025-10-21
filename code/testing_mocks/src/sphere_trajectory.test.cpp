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

    Eigen::Vector3d const camera_position_x{1, 0, 0};
    Eigen::Vector3d const x{TrackPoint(world_origin, camera_position_x)};
    EXPECT_TRUE(x.isApprox(Eigen::Vector3d{0, -M_PI / 2, 0}));

    Eigen::Vector3d const camera_position_y{0, 1, 0};
    Eigen::Vector3d const y{TrackPoint(world_origin, camera_position_y)};
    EXPECT_TRUE(y.isApprox(Eigen::Vector3d{M_PI / 2, 0, 0}));

    // WARN(Jack): This test actually checks that a known unhandled error happens like we expect. When the origin
    // world_origin and camera_position are in the same xy plane we get nans! See note in implementation for more
    // detail.
    Eigen::Vector3d const camera_position_z{0, 0, 1};
    Eigen::Vector3d const z{TrackPoint(world_origin, camera_position_z)};
    EXPECT_TRUE(z.array().isNaN().all());
}