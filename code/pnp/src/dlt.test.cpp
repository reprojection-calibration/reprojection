#include "dlt.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "multiple_view_geometry_data_generator.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection;
using namespace reprojection::pnp;

TEST(PnpDlt, TestDlt23) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};
        auto const [tf, K]{Dlt23(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Eigen::Vector<double, 6> const pose_i{geometry::Log(tf)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose)) << "Result:\n" << pose_i << "\nexpected result:\n" << frame_i.pose;

        EXPECT_TRUE(K.isUpperTriangular());  // Property of camera intrinsic matrix
        EXPECT_TRUE(K.isApprox(generator.GetK()));
    }
}

TEST(PnpDlt, TestDlt22) {
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};       // Pixels must be in normalized space for Dlt22
    MvgFrameGenerator const generator{MvgFrameGenerator(true, K)};  // Points must have Z=0 (flat = true)

    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};

        auto const tf{Dlt22(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Eigen::Vector<double, 6> const pose_i{geometry::Log(tf)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose)) << "Result:\n" << pose_i << "\nexpected result:\n" << frame_i.pose;
    }
}

// TODO REMOVE
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

// TODO REMOVE
TEST(PnpHomographyDecomposition, TestDlt22) {
    Eigen::MatrixX2d const pixels{{0, 0}, {2, 2}, {-2, -2}, {-2, 2}, {2, -2}};
    Eigen::MatrixX3d const points{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};

    Eigen::Isometry3d const T{Dlt22(pixels, points)};

    EXPECT_TRUE(T.matrix().diagonal().isApproxToConstant(1));
    EXPECT_FLOAT_EQ(T.translation()(2), 0.5);
}