#include "homography_decomposition.hpp"

#include <gtest/gtest.h>

#include "matrix_utilities.hpp"

using namespace reprojection::pnp;

void PrepForHomographySolving(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    (void)pixels;
    auto const [_, V]{WhatDoWeNameThis(points)};

    Eigen::Matrix3d R{V};
    if (V(0, 2) * V(0, 2) + V(1, 2) * V(1, 2) < 1e-10) {
        R = Eigen::Matrix3d::Identity();
    } else if (R.determinant() < 0) {
        R *= -1;
    }

    Eigen::Vector3d const center{points.colwise().mean()};
    Eigen::Vector3d const t{-R * center};

    Eigen::Isometry3d const T{ToIsometry3d(R, t)};

    Eigen::MatrixX2d const normalized_chopped_points{
        (T * points.rowwise().homogeneous().transpose()).transpose()(Eigen::all, {0, 1})};

    std::cout << R << std::endl;
    std::cout << t << std::endl;
    std::cout << normalized_chopped_points << std::endl;
}

TEST(PnpHomographyDecomposition, TestYYY) {
    Eigen::MatrixX2d const pixels{};  // ADD REAL PIXELS

    // Any three points are on a plane!
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    PrepForHomographySolving(pixels, three_points);

    // z=0 plane
    Eigen::MatrixX3d plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    PrepForHomographySolving(pixels, plane);
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
