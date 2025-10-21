#include "eigen_utilities/grid.hpp"

namespace reprojection::eigen_utilities {

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

Eigen::ArrayXi ToEigen(std::vector<int> const& vector) {
    return Eigen::Map<Eigen::ArrayXi const>(vector.data(), std::size(vector));
}

}  // namespace reprojection::eigen_utilities