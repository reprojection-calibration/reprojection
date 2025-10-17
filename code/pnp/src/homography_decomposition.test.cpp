#include "homography_decomposition.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "multiple_view_geometry_data_generator.hpp"

using namespace reprojection;
using namespace reprojection::pnp;

TEST(PnpDlt, TestDlt) {
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // Pixels must be in normalized space
    MvgFrameGenerator const generator{MvgFrameGenerator(true, K)};

    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};

        auto const tf{FullPipeline(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Eigen::Vector<double, 6> const pose_i{geometry::Log(tf)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose)) << "Result:\n" << pose_i << "\nexpected result:\n" << frame_i.pose;
    }
}

TEST(PnpHomographyDecomposition, TestFullPipelineOpenCvData) {
    double const L = 0.2;
    Eigen::MatrixX3d const points_w{
        {-L, -L, 0}, {2 * L, -L, 0}, {L, L, 0}, {-L, L, 0}, {-L, 2 * L, 0}};  // Points should always have z=0
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};                 // Pixels should be in normalized space

    Eigen::Isometry3d tf_co_w{reprojection::geometry::Exp(
        Eigen::Vector<double, 6>{M_PI / 180 * (5), M_PI / 180 * (0), M_PI / 180 * (45), -0.1, 0.1, 1.2})};
    Eigen::MatrixX2d const pixels{MvgFrameGenerator::Project(points_w, K, tf_co_w)};

    auto const tf{FullPipeline(pixels, points_w)};

    EXPECT_FLOAT_EQ(tf.linear().diagonal().sum(), 2.4073617);  // HEURSITIC
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{-0.1, 0.1, 1.2}));
}

TEST(PnpHomographyDecomposition, TestFullPipeline) {
    Eigen::MatrixX2d const pixels{{0, 0}, {2, 2}, {-2, -2}, {-2, 2}, {2, -2}};
    Eigen::MatrixX3d const points{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};

    Eigen::Isometry3d const T{FullPipeline(pixels, points)};

    EXPECT_TRUE(T.matrix().diagonal().isApproxToConstant(1));
    EXPECT_FLOAT_EQ(T.translation()(2), 0.5);
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
    auto const [normalized_three_points, tf0]{NormalizePointsForHomographySolving(three_points)};
    EXPECT_TRUE(normalized_three_points.isApprox(Eigen::MatrixX2d{{-0.538005, 0.876209}, {0, 0}, {0.538005, -0.876209}},
                                                 1e-6));  // Heuristic
    EXPECT_TRUE((tf0.linear() * tf0.linear().transpose()).isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(tf0.translation().isApprox(Eigen::Vector3d{-1.07601, 1.75242, 2.78769}, 1e-5));  // HEURISTIC!

    // z=0 plane
    Eigen::MatrixX3d const z_zero_plane{{0, 0, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    auto const [normalized_z_zero_plane, tf1]{NormalizePointsForHomographySolving(z_zero_plane)};
    EXPECT_TRUE(
        normalized_z_zero_plane.isApprox(Eigen::MatrixX2d{{0, 0}, {1, 1}, {-1, -1}, {-1, 1}, {1, -1}}));  // Heuristic
    EXPECT_TRUE((tf1.linear() * tf1.linear().transpose()).isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(tf1.translation().isApprox(Eigen::Vector3d{0, 0, 0}));  // HEURISTIC!
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
