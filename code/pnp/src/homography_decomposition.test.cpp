#include "homography_decomposition.hpp"

#include <gtest/gtest.h>

#include "dlt.hpp"
#include "geometry/lie.hpp"
#include "multiple_view_geometry_data_generator.hpp"

using namespace reprojection;
using namespace reprojection::pnp;

TEST(PnpHomographyDecomposition, TestDlt22OpenCvData) {
    double const L = 0.2;
    Eigen::MatrixX3d const points_w{
        {-L, -L, 0}, {2 * L, -L, 0}, {L, L, 0}, {-L, L, 0}, {-L, 2 * L, 0}};  // Points should always have z=0
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};                 // Pixels should be in normalized space

    Eigen::Isometry3d tf_co_w{reprojection::geometry::Exp(
        Eigen::Vector<double, 6>{M_PI / 180 * (5), M_PI / 180 * (0), M_PI / 180 * (45), -0.1, 0.1, 1.2})};
    Eigen::MatrixX2d const pixels{MvgFrameGenerator::Project(points_w, K, tf_co_w)};

    auto const tf{Dlt22(pixels, points_w)};

    EXPECT_FLOAT_EQ(tf.linear().diagonal().sum(), 2.4073617);  // HEURSITIC
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{-0.1, 0.1, 1.2}));
}

TEST(PnpHomographyDecomposition, TestDlt22) {
    Eigen::MatrixX2d const pixels{{0, 0}, {2, 2}, {-2, -2}, {-2, 2}, {2, -2}};
    Eigen::MatrixX3d const points{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};

    Eigen::Isometry3d const T{Dlt22(pixels, points)};

    EXPECT_TRUE(T.matrix().diagonal().isApproxToConstant(1));
    EXPECT_FLOAT_EQ(T.translation()(2), 0.5);
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
