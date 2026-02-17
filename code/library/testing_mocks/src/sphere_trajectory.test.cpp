#include "sphere_trajectory.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

// WARN(Jack): In this test we do not test that the direction the tfs are pointing at is correct! Only the positions of
// them.
TEST(TestingMocksSphereTrajectory, TestSphereTrajectory) {
    int const num_camera_poses{100};
    double const sphere_radius{1.5};
    Vector3d const sphere_origin{1, 2, 3};
    testing_mocks::CameraTrajectory const config{{0, 0, 0}, sphere_radius, sphere_origin};

    std::vector<Vector6d> const tfs{testing_mocks::SphereTrajectory(num_camera_poses, config)};

    double radius{0};
    Vector3d centroid{0, 0, 0};
    for (auto const& tf : tfs) {
        radius += (tf.bottomRows(3) - sphere_origin).norm();
        centroid += tf.bottomRows(3);
    }
    radius = radius / std::size(tfs);
    centroid = centroid / std::size(tfs);

    EXPECT_FLOAT_EQ(radius, sphere_radius);
    EXPECT_TRUE(centroid.isApprox(sphere_origin));
}

TEST(TestingMocksSphereTrajectory, TestTrackPoint) {
    Vector3d const world_origin{0, 0, 0};
    Matrix3d const R{Matrix3d::Identity()};
    Vector3d const forward{Vector3d::Zero()};

    auto const [R_new, forward_new]{testing_mocks::TrackPoint(world_origin, world_origin, R, forward)};
    EXPECT_TRUE(R_new.isApprox(Matrix3d::Identity()));
    EXPECT_TRUE(forward_new.isApprox(Vector3d::Zero()));
}