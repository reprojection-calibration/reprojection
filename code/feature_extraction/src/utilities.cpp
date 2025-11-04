#include "utilities.hpp"

namespace reprojection::feature_extraction {

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