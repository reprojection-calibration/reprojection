#include "utilities.hpp"

namespace reprojection::feature_extraction {

cv::Mat ToGray(cv::Mat const& img) {
    cv::Mat img_8u;
    if (img.depth() != CV_8U) {
        double minVal, maxVal;
        cv::minMaxLoc(img, &minVal, &maxVal);
        img.convertTo(img_8u, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    } else {
        img_8u = img;
    }

    cv::Mat img_gray;
    if (img_8u.channels() == 3) {
        cv::cvtColor(img_8u, img_gray, cv::COLOR_BGR2GRAY);
    } else if (img_8u.channels() == 4) {
        cv::cvtColor(img_8u, img_gray, cv::COLOR_BGRA2GRAY);
    } else if (img_8u.channels() == 1) {
        img_gray = img_8u;
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR: Unsupported number of channels: " +  // LCOV_EXCL_LINE
                                 std::to_string(img_8u.channels()));                                 // LCOV_EXCL_LINE
    }

    return img_gray;
}

MatrixX2d ToEigen(std::vector<cv::Point2f> const& points) {
    MatrixX2d eigen_points(std::size(points), 2);
    for (Eigen::Index i = 0; i < eigen_points.rows(); i++) {
        eigen_points.row(i)[0] = points[i].x;
        eigen_points.row(i)[1] = points[i].y;
    }

    return eigen_points;
}  // LCOV_EXCL_LINE

double AlternatingSum(int const n, double const increment_1, double const increment_2) {
    double sum{0};
    for (int i{0}; i < n; ++i) {
        sum += (i % 2 == 0) ? increment_1 : increment_2;
    }

    return sum;
}

}  // namespace reprojection::feature_extraction