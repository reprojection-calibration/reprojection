#pragma once

#include "types/eigen_types.hpp"

#include <opencv2/opencv.hpp>

namespace reprojection::feature_extraction {

cv::Mat GenerateCheckerboard(cv::Size const& pattern_size, int const square_size_pixels);

cv::Mat GenerateCircleGrid(cv::Size const& pattern_size, int const circle_radius_pixels,
                           int const circle_spacing_pixels, bool const asymmetric);

struct AprilBoard3Generation {
    static cv::Mat GenerateBoard(int const num_bits, uint64_t const tag_family[], int const bit_size_pixels,
                                 cv::Size const& pattern_size);

    static cv::Mat GenerateTag(int const num_bits, uint64_t const tag_code, int const bit_size_pixels);

    static cv::Mat GenerateTag(int const bit_size_pixels, MatrixXi const& code_matrix);

    static MatrixXi GenerateCodeMatrix(int const num_bits, uint64_t const tag_code);

    static MatrixXi Rotate90(MatrixXi const& matrix, bool const clockwise = false);
};

}  // namespace reprojection::feature_extraction