#include "eigen_utilities/grid.hpp"

namespace reprojection::eigen_utilities {

ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only) {
    ArrayXi const row_indices{ArrayXi::LinSpaced(rows * cols, 0, rows - 1)};
    ArrayXi const col_indices{ArrayXi::LinSpaced(cols, 0, cols).colwise().replicate(rows)};

    ArrayX2i grid_indices(rows * cols, 2);
    grid_indices.col(0) = row_indices;
    grid_indices.col(1) = col_indices;

    if (even_only) {
        // NOTE(Jack): Eigen does not provide direct way to apply the modulo operator, so we follow a method using a
        // unaryExpr() that we adopted from here
        // (https://stackoverflow.com/questions/35798698/eigen-matrix-library-coefficient-wise-modulo-operation)
        ArrayXb const is_even_mask{((grid_indices.rowwise().sum().unaryExpr([](int const x) { return x % 2; })) == 0)};
        ArrayXi const even_row_ids{MaskToRowId(is_even_mask)};

        return grid_indices(even_row_ids, Eigen::all);
    }

    return grid_indices;
}

// There has to be a more eloquent way to do this... but it gets the job done :)
ArrayXi MaskToRowId(ArrayXb const& mask) {
    std::vector<int> indices;
    indices.reserve(mask.rows());

    for (Eigen::Index i{0}; i < mask.rows(); i++) {
        if (mask(i) == true) {
            indices.push_back(i);
        }
    }

    return ToEigen(indices);
}

ArrayXi ToEigen(std::vector<int> const& vector) { return Eigen::Map<ArrayXi const>(vector.data(), std::size(vector)); }

}  // namespace reprojection::eigen_utilities