#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace reprojection_calibration::feature_extraction {

// Requesting even_only has the effect of allowing us to produce asymmetric grids like those required by the asymmetric
// circle grid target.
Eigen::ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only = false);

Eigen::MatrixX2d ToEigen(std::vector<cv::Point2f> const& points);

Eigen::ArrayXi ToEigen(std::vector<int> const& vector);

Eigen::ArrayXi MaskIndices(Eigen::ArrayXi const& array);

double AlternatingSum(int const n, double const increment_1, double const increment_2);

}  // namespace reprojection_calibration::feature_extraction