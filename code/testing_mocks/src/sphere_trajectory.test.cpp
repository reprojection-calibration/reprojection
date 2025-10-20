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