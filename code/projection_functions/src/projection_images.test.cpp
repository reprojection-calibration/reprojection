#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "eigen_utilities/camera.hpp"
#include "eigen_utilities/grid.hpp"
#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"

using namespace reprojection;
using namespace reprojection::projection_functions;

TEST(ProjectionFunctionsProjectionImage, TestPinholeProjection) {
    int const width{720};
    int const height{480};
    Eigen::Array<double, 4, 1> const pinhole_intrinsics{600, 600, width / 2, height / 2};

    int const grid_size{70};
    Eigen::ArrayX2i const grid_2d{eigen_utilities::GenerateGridIndices(grid_size, grid_size)};
    Eigen::MatrixX3d grid_3d(grid_2d.rows(), 3);
    grid_3d.leftCols(2) = grid_2d.cast<double>() - grid_size / 2;
    grid_3d.col(2).setConstant(100);

    MatrixX2d const pixels{PinholeProjection(eigen_utilities::ToK(pinhole_intrinsics), grid_3d)};

    cv::Mat img = 255 * cv::Mat::ones(cv::Size(width, height), CV_8UC1);

    for (int i{0}; i < pixels.rows(); ++i) {
        auto const pixel_i{pixels.row(i)};
        img.at<uchar>(pixel_i(1), pixel_i(0)) = 0;
        img.at<uchar>(pixel_i(1) + 1, pixel_i(0) + 1) = 0;
        img.at<uchar>(pixel_i(1) + 1, pixel_i(0) - 1) = 0;
        img.at<uchar>(pixel_i(1) - 1, pixel_i(0) + 1) = 0;
        img.at<uchar>(pixel_i(1) - 1, pixel_i(0) - 1) = 0;
    }

    cv::imwrite("projection_pinhole.png", img);

    EXPECT_TRUE(false);
}

TEST(ProjectionFunctionsProjectionImage, TestDoubleSphereProjection) {
    int const width{720};
    int const height{480};

    Eigen::Array<double, 6, 1> const double_sphere_intrinsics{600, 600, width / 2, height / 2, -0.5, -0.5};

    int const grid_size{70};
    Eigen::ArrayX2i const grid_2d{eigen_utilities::GenerateGridIndices(grid_size, grid_size)};
    Eigen::MatrixX3d grid_3d(grid_2d.rows(), 3);
    grid_3d.leftCols(2) = grid_2d.cast<double>() - grid_size / 2;
    grid_3d.col(2).setConstant(100);

    cv::Mat img = 255 * cv::Mat::ones(cv::Size(width, height), CV_8UC1);

    for (int i{0}; i < grid_3d.rows(); ++i) {
        Eigen::Vector2d const pixel_i(DoubleSphereProjection<double>(double_sphere_intrinsics, grid_3d.row(i)));
        if (pixel_i(0) < 1 or pixel_i(0) > width - 2 or pixel_i(1) < 1 or pixel_i(1) > height - 2) {
            continue;
        }
        img.at<uchar>(pixel_i(1), pixel_i(0)) = 0;
        img.at<uchar>(pixel_i(1) + 1, pixel_i(0) + 1) = 0;
        img.at<uchar>(pixel_i(1) + 1, pixel_i(0) - 1) = 0;
        img.at<uchar>(pixel_i(1) - 1, pixel_i(0) + 1) = 0;
        img.at<uchar>(pixel_i(1) - 1, pixel_i(0) - 1) = 0;
    }

    cv::imwrite("projection_double_sphere.png", img);

    EXPECT_TRUE(false);
}

TEST(ProjectionFunctionsProjectionImage, TestPinholeRadtan4Projection) {
    int const width{720};
    int const height{480};

    Eigen::Array<double, 8, 1> const pinhole_radtan4_intrinsics{600, 600, 360, 240, 0.5, 0.5, 0.1, 0.1};

    int const grid_size{70};
    Eigen::ArrayX2i const grid_2d{eigen_utilities::GenerateGridIndices(grid_size, grid_size)};
    Eigen::MatrixX3d grid_3d(grid_2d.rows(), 3);
    grid_3d.leftCols(2) = grid_2d.cast<double>() - grid_size / 2;
    grid_3d.col(2).setConstant(100);

    cv::Mat img = 255 * cv::Mat::ones(cv::Size(width, height), CV_8UC1);

    for (int i{0}; i < grid_3d.rows(); ++i) {
        Eigen::Vector2d const pixel_i(PinholeRadtan4Projection<double>(pinhole_radtan4_intrinsics, grid_3d.row(i)));
        img.at<uchar>(pixel_i(1), pixel_i(0)) = 0;
        img.at<uchar>(pixel_i(1) + 1, pixel_i(0) + 1) = 0;
        img.at<uchar>(pixel_i(1) + 1, pixel_i(0) - 1) = 0;
        img.at<uchar>(pixel_i(1) - 1, pixel_i(0) + 1) = 0;
        img.at<uchar>(pixel_i(1) - 1, pixel_i(0) - 1) = 0;
    }

    cv::imwrite("projection_pinhole_radtan4.png", img);

    EXPECT_TRUE(false);
}