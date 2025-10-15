#include "utilities.hpp"

#include <vector>

namespace reprojection_calibration::feature_extraction {

Eigen::ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only) {
    Eigen::ArrayXi const row_indices{Eigen::ArrayXi::LinSpaced(rows * cols, 0, rows - 1)};
    Eigen::ArrayXi const col_indices{Eigen::ArrayXi::LinSpaced(cols, 0, cols).colwise().replicate(rows)};

    Eigen::ArrayX2i grid_indices(rows * cols, 2);
    grid_indices.col(0) = row_indices;
    grid_indices.col(1) = col_indices;

    if (even_only) {
        // NOTE(Jack): Eigen does not provide direct way to apply the modulo operator, so we follow a method using a
        // unaryExpr() that we adopted from here
        // (https://stackoverflow.com/questions/35798698/eigen-matrix-library-coefficient-wise-modulo-operation)
        Eigen::ArrayXi const is_even{
            ((grid_indices.rowwise().sum().unaryExpr([](int const x) { return x % 2; })) == 0).cast<int>()};
        Eigen::ArrayXi const mask{MaskIndices(is_even)};

        return grid_indices(mask, Eigen::all);
    }

    return grid_indices;
}

Eigen::MatrixX2d ToEigen(std::vector<cv::Point2f> const& points) {
    Eigen::MatrixX2d eigen_points(std::size(points), 2);
    for (Eigen::Index i = 0; i < eigen_points.rows(); i++) {
        eigen_points.row(i)[0] = points[i].x;
        eigen_points.row(i)[1] = points[i].y;
    }

    return eigen_points;
}

Eigen::ArrayXi ToEigen(std::vector<int> const& vector) {
    return Eigen::Map<Eigen::ArrayXi const>(vector.data(), std::size(vector));
}

// There has to be a more eloquent way to do this... but it gets the job done :)
Eigen::ArrayXi MaskIndices(Eigen::ArrayXi const& array) {
    std::vector<int> mask;
    mask.reserve(array.rows());

    for (Eigen::Index i{0}; i < array.rows(); i++) {
        if (array(i) == 1) {
            mask.push_back(i);
        }
    }

    return ToEigen(mask);
}

double AlternatingSum(int const n, double const increment_1, double const increment_2) {
    double sum{0};
    for (int i{0}; i < n; ++i) {
        sum += (i % 2 == 0) ? increment_1 : increment_2;
    }

    return sum;
}

}  // namespace reprojection_calibration::feature_extraction