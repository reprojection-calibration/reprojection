#include "homography_decomposition.hpp"

#include <gtest/gtest.h>

#include "dlt.hpp"
#include "geometry/lie.hpp"

using namespace reprojection::pnp;

// NOTE(Jack): We should be able to use the same normalization here we used for the DLT
// pixels, 2d_points
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> FindHomography(Eigen::MatrixX2d const& points_src,
                                                            Eigen::MatrixX2d const& points_dst) {
    auto const A{ConstructA<3>(points_src, points_dst)};
    auto H{SolveForP<3>(A)};
    H /= H(2, 2);

    Eigen::Vector3d const h_norms{H.colwise().norm()};

    H.col(0) = H.col(0) / h_norms(0);
    H.col(1) = H.col(1) / h_norms(1);
    Eigen::Vector3d const t{H.col(2) * (2.0 / (h_norms(0) + h_norms(1)))};
    H.col(2) = H.col(0).cross(H.col(1));

    Eigen::Matrix3d const cleaned_H{reprojection::geometry::Exp((reprojection::geometry::Log(H)))};

    return {t, cleaned_H};
}

TEST(PnpHomographyDecomposition, TestFindHomography) {
    // Same points for src and dst
    Eigen::MatrixX2d const points1{{0, 0}, {1, 1}, {-1, -1}, {-1, 1}, {1, -1}};
    auto const [t1, R1]{FindHomography(points1, points1)};
    EXPECT_TRUE(t1.isApprox(Eigen::Vector3d{0, 0, 1}));
    EXPECT_TRUE(R1.isApprox(Eigen::Matrix3d::Identity()));

    // Different points for src and dst
    Eigen::MatrixX2d const points2{2 * points1};
    auto const [t2, R2]{FindHomography(points1, points2)};
    EXPECT_TRUE(t2.isApprox(Eigen::Vector3d{0, 0, 2}));
    EXPECT_TRUE(R2.isApprox(Eigen::Matrix3d::Identity()));
}

TEST(PnpHomographyDecomposition, TestNormalizePointsForHomographySolving) {
    // Any three non-colinear points are on a plane!
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    Eigen::MatrixX2d const normalized_three_points{NormalizePointsForHomographySolving(three_points)};
    EXPECT_TRUE(normalized_three_points.isApprox(Eigen::MatrixX2d{{-0.538005, 0.876209}, {0, 0}, {0.538005, -0.876209}},
                                                 1e-6));  // Heuristic

    // z=0 plane
    Eigen::MatrixX3d z_zero_plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    Eigen::MatrixX2d const normalized_z_zero_plane{NormalizePointsForHomographySolving(z_zero_plane)};
    EXPECT_TRUE(
        normalized_z_zero_plane.isApprox(Eigen::MatrixX2d{{0, 0}, {1, 1}, {-1, -1}, {-1, 1}, {1, -1}}));  // Heuristic
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
