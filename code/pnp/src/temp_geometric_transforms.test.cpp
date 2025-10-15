#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"
#include "nonlinear_refinement.hpp"
#include "pose_utilities.hpp"

// WARN(Jack): This test shows us that we currently have two working implementations of the geometric transforms and
// pinhole projection, which means we have copy and pasted ideas/code. For now I will leave this comparison test to make
// sure our implementations are consistent (even though for the tfs it does not really mean that as we copy and pasted
// the logic), and as a reminder that we need to solve this problem.

using namespace reprojection::pnp;

TEST(PnpTempGeometricTransformsTest, Test3DTransformation) {
    Se3 const tf_slash_pose{ceres::constants::pi, ceres::constants::pi, ceres::constants::pi, 1, -2, 3};
    Eigen::Vector3d const point{1, 2, 3};

    Eigen::Vector3d const ceres_point{TransformPoint<double>(tf_slash_pose, point)};

    Eigen::Isometry3d const tf_co_w{FromSe3(tf_slash_pose)};
    Eigen::Vector3d const custom_point{(tf_co_w * point.homogeneous())};  // DANGER - COPY PASTED LOGIC!!!

    EXPECT_TRUE(ceres_point.isApprox(custom_point));
}

TEST(PnpTempGeometricTransformsTest, TestPinholeProjection) {
    Eigen::MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                    {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    Eigen::Matrix3d const K{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    Eigen::Isometry3d const tf_co_w{Eigen::Isometry3d::Identity()};
    Eigen::MatrixX2d const pixels{MvgFrameGenerator::Project(points_w, K, tf_co_w)};

    Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                       {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};
    ASSERT_TRUE(pixels.isApprox(test_pixels, 1e-3));

    Eigen::Array<double, 4, 1> const pinhole_intrinsics{600, 600, 360, 240};
    for (Eigen::Index i{0}; i < points_w.rows(); ++i) {
        Eigen::Vector2d const nlr_pixel = PinholeProjection<double>(pinhole_intrinsics.data(), points_w.row(i));
        ASSERT_TRUE(nlr_pixel.isApprox(test_pixels.row(i).transpose(), 1e-3));
    }
}
