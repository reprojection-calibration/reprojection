#include "multiple_view_geometry_data_generator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

using namespace reprojection_calibration::pnp;

TEST(MultipleViewGeometryDataGenerator, TestMvgFrameGenerator) {
    MvgFrameGenerator const test_data_generator{MvgFrameGenerator()};
    EXPECT_NO_THROW(test_data_generator.Generate());
}

TEST(MultipleViewGeometryDataGenerator, TestTrackPoint) {
    // 3D coordinate viewer: https://dugas.ch/transform_viewer/index.html
    Eigen::Vector3d tracking_direction{MvgFrameGenerator::TrackPoint({0, 0, 0}, {2, 0, 0})};
    EXPECT_TRUE(tracking_direction.isApprox(Eigen::Vector3d{0, -EIGEN_PI / 2.0, 0}));

    tracking_direction = MvgFrameGenerator::TrackPoint({0, 0, 0}, {0, 2, 0});
    EXPECT_TRUE(tracking_direction.isApprox(Eigen::Vector3d{EIGEN_PI / 2.0, 0, 0}));

    tracking_direction = MvgFrameGenerator::TrackPoint({0, 0, 0}, {2, 2, 2});
    EXPECT_TRUE(tracking_direction.isApprox(Eigen::Vector3d{1.54593, -1.54593, 0}, 1e-4));  // Heuristic
}

TEST(MultipleViewGeometryDataGenerator, TestProject) {
    Eigen::MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                    {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    Eigen::Matrix3d const K{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    Eigen::Isometry3d const tf_co_w{Eigen::Isometry3d::Identity()};
    Eigen::MatrixX2d const pixels{MvgFrameGenerator::Project(points_w, K, tf_co_w)};

    Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                       {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};
    ASSERT_TRUE(pixels.isApprox(test_pixels, 1e-3));
}
