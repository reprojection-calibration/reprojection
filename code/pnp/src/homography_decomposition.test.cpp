#include "homography_decomposition.hpp"

#include <gtest/gtest.h>

using namespace reprojection::pnp;

void FindHomography(Eigen::MatrixX2d const& points_src, Eigen::MatrixX2d const& points_dst) {
    (void)points_dst;
    (void)points_src;
}

TEST(PnpHomographyDecomposition, TestFindHomography) {
    Eigen::MatrixX2d const points1{{0, 0}, {1, 1}, {-1, -1}, {-1, 1}, {1, -1}};
    FindHomography(points1, points1);

    Eigen::MatrixX2d const points2{2 * points1};
    FindHomography(points1, points2);

    EXPECT_FALSE(true);
}

TEST(PnpHomographyDecomposition, TestNormalizePointsForHomographySolving) {
    // Any three non-colinear points are on a plane!
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    Eigen::MatrixX2d const normalized_three_points{NormalizePointsForHomographySolving(three_points)};
    EXPECT_TRUE(normalized_three_points.isApprox(Eigen::MatrixX2d{{-0.538005, 0.876209}, {0, 0}, {0.538005, -0.876209}},
                                                 1e-6));  // Heurstic

    // z=0 plane
    Eigen::MatrixX3d z_zero_plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    Eigen::MatrixX2d const normalized_z_zero_plane{NormalizePointsForHomographySolving(z_zero_plane)};
    EXPECT_TRUE(
        normalized_z_zero_plane.isApprox(Eigen::MatrixX2d{{0, 0}, {1, 1}, {-1, -1}, {-1, 1}, {1, -1}}));  // Heurstic
}

TEST(PnpHomographyDecomposition, TestXXX) {
    // Any three points are on a plane!
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    EXPECT_TRUE(IsPlane(three_points));

    // z=0 plane
    Eigen::MatrixX3d plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    EXPECT_TRUE(IsPlane(plane));

    // Make an outlier
    plane(0, 2) = 10;
    EXPECT_FALSE(IsPlane(plane));
}
