#include "pose_utilities.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

using namespace reprojection::pnp;

// TODO(Jack): Replace with methods from geometry/lie
TEST(PnpPoseUtilities, TestToSe3) {
    Eigen::Isometry3d const identity{Eigen::Isometry3d::Identity()};
    EXPECT_FLOAT_EQ(ToSe3(identity).sum(), 0.0);

    // Test with two different rotations
    Eigen::Isometry3d const yaw_rotation{Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitZ())};
    EXPECT_TRUE(ToSe3(yaw_rotation).isApprox(Se3{0, 0, 0.5 * EIGEN_PI, 0, 0, 0}));

    Eigen::Isometry3d const pitch_yaw_rotation{Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitZ())};
    EXPECT_TRUE(ToSe3(pitch_yaw_rotation).isApprox(Se3{1.2092, 1.2092, 1.2092, 0, 0, 0}, 1e-3));  // Heuristic result

    // Tests with translation only
    Eigen::Isometry3d translation{Eigen::Isometry3d::Identity()};
    translation.translation() = Eigen::Vector3d(-1, 0, 1);
    EXPECT_TRUE(ToSe3(translation).isApprox(Se3{0, 0, 0, -1, 0, 1}));

    // Test with a translation and rotation
    Eigen::Isometry3d translation_and_rotation{Eigen::Isometry3d::Identity()};
    translation_and_rotation.translation() = Eigen::Vector3d(-1, 0, 1);
    translation_and_rotation.rotate(Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitZ()));
    EXPECT_TRUE(ToSe3(translation_and_rotation).isApprox(Se3{0, 0, 0.5 * EIGEN_PI, -1, 0, 1}));
}

TEST(PnpPoseUtilities, TestFromSe3) {
    Se3 const identity{0, 0, 0, 0, 0, 0};
    EXPECT_FLOAT_EQ(FromSe3(identity).matrix().diagonal().sum(), 4.0);

    // Test the reversability - this is easier than trying to engineer some SE3 input matrices
    Se3 const rotations{1, 1, 1, 0, 0, 0};
    EXPECT_TRUE(ToSe3(FromSe3(rotations)).isApprox(rotations));

    Se3 const translations{0, 0, 0, 1, 1, 1};
    EXPECT_TRUE(ToSe3(FromSe3(translations)).isApprox(translations));

    Se3 const rotations_and_translations{1, 1, 1, 1, 1, 1};
    EXPECT_TRUE(ToSe3(FromSe3(rotations_and_translations)).isApprox(rotations_and_translations));
}
