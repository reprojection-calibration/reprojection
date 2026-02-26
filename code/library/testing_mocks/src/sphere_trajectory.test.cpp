#include "sphere_trajectory.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// WARN(Jack): In this test we do not test that the direction the tfs are pointing at is correct! Only the positions of
// them.
TEST(TestingMocksSphereTrajectory, TestSphereTrajectory) {
    int const num_camera_poses{100};
    double const sphere_radius{1.5};
    Vector3d const sphere_origin{1, 2, 3};
    testing_mocks::CameraTrajectory const config{{0, 0, 0}, sphere_radius, sphere_origin};

    std::vector<Vector6d> const poses{testing_mocks::SphereTrajectory(num_camera_poses, config)};

    double radius{0};
    Vector3d centroid{0, 0, 0};
    for (auto const& aa_w_co : poses) {
        radius += (aa_w_co.tail<3>() - sphere_origin).norm();
        centroid += aa_w_co.tail<3>();
    }
    radius = radius / std::size(poses);
    centroid = centroid / std::size(poses);

    EXPECT_FLOAT_EQ(radius, sphere_radius);
    EXPECT_TRUE(centroid.isApprox(sphere_origin));
}

// TODO(Jack): To be perfectly honest I am lacking a strong intuition of the math here, and am so tired of debugging it
//  manually that I have no desire to engineer more testing here. Therefore I just check one kind of random set of input
//  parameters here, but the test is (and methods in general) are not so easy to understand by inspection, which tells
//  me that one day we can improve these!
TEST(TestingMocksSphereTrajectory, TestTrackPoint) {
    Vector3d const world_origin{0, 0, 0};
    Vector3d const camera_position{0, 0, 2};
    Matrix3d const R{Matrix3d::Identity()};
    Vector3d const forward{Vector3d::UnitX()};

    auto [R_new, forward_new]{testing_mocks::TrackPoint(world_origin, camera_position, R, forward)};
    EXPECT_TRUE(R_new.isApprox(Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitY()).toRotationMatrix()));
    EXPECT_TRUE(forward_new.isApprox(Vector3d{0, 0, -1}));
}

TEST(TestingMocksSphereTrajectory, TestTrackPointShortCiruits) {
    Vector3d const world_origin{0, 0, 0};
    Matrix3d const R{Matrix3d::Random()};  // Not a canonical rotation matrix! It's just random numbers.
    Vector3d forward{Vector3d::Random()};

    // World origin and camera are the same position - should short circuit and just return the previous rotation
    // and forward direction vector.
    auto [R_new, forward_new]{testing_mocks::TrackPoint(world_origin, world_origin, R, forward)};
    EXPECT_TRUE(R_new.isApprox(R));
    EXPECT_TRUE(forward_new.isApprox(forward));

    // When the two forward directions are the same we should short circuit and just return the inputs.
    Vector3d camera_position{2, 2, 2};
    forward = world_origin - camera_position;
    std::tie(R_new, forward_new) = testing_mocks::TrackPoint(world_origin, camera_position, R, forward);
    EXPECT_TRUE(R_new.isApprox(R));
    EXPECT_TRUE(forward_new.isApprox(forward));
}