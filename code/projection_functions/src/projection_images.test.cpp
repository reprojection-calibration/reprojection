#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "eigen_utilities/camera.hpp"
#include "eigen_utilities/grid.hpp"
#include "projection_functions/pinhole.hpp"

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

    grid_3d.col(2).setConstant(100);  // Points 10m in front of camera

    MatrixX2d const pixels{PinholeProjection(eigen_utilities::ToK(pinhole_intrinsics), grid_3d)};

    cv::Mat img = 255*cv::Mat::ones(cv::Size(width, height), CV_8UC1);

    for (int i{0}; i < pixels.rows(); ++i) {
        auto const pixel_i{pixels.row(i)};
        img.at<uchar>(pixel_i(1), pixel_i(0)) = 0;
    }

    cv::imwrite("pinhole_projection.png", img);

    EXPECT_TRUE(false);
}
