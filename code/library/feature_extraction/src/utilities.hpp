#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "types/eigen_types.hpp"

namespace reprojection::feature_extraction {

MatrixX2d ToEigen(std::vector<cv::Point2f> const& points);

double AlternatingSum(int const n, double const increment_1, double const increment_2);

}  // namespace reprojection::feature_extraction