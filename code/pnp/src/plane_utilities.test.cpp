#include "plane_utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection::pnp;

TEST(PnpHomographyDecomposition, TestIsPlane) {
    // Any three points are on a plane! Wait... but these are on a line! This is a poorly handled edge case of our
    // implementation.
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    EXPECT_TRUE(IsPlane(three_points));

    // z=0 plane
    Eigen::MatrixX3d plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    EXPECT_TRUE(IsPlane(plane));

    // Make an outlier
    plane(0, 2) = 10;
    EXPECT_FALSE(IsPlane(plane));
}

TEST(PnpHomographyDecomposition, TestPca) {
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};

    // "svs" = singular values
    // "evs" = eigen vectors (is that right ?)
    auto const [svs1, evs1]{Pca(three_points)};
    EXPECT_TRUE(svs1.isApprox(Eigen::Vector3d{6, 0, 0}));  // Indicates collinearity?
    EXPECT_EQ(evs1(2, 2), 0);  // Is this a singularity/artefact from points being collinear?

    // z=0 plane
    Eigen::MatrixX3d const plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};

    auto const [svs2, evs2]{Pca(plane)};
    EXPECT_TRUE(svs2.isApprox(Eigen::Vector3d{4, 4, 0}));
    EXPECT_TRUE(evs2.isApprox(Eigen::Matrix3d::Identity()));
}
