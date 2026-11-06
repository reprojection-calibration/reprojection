#include "eigen_utilities/grid.hpp"

namespace reprojection::eigen_utilities {

ArrayX2i GenerateGridIndices(int const rows, int const cols) {
    ArrayXi const row_indices{ArrayXi::LinSpaced(rows * cols, 0, rows - 1)};
    ArrayXi const col_indices{ArrayXi::LinSpaced(cols, 0, cols).colwise().replicate(rows)};

    ArrayX2i grid_indices(rows * cols, 2);
    grid_indices.col(0) = row_indices;
    grid_indices.col(1) = col_indices;

    return grid_indices;
}

// There has to be a more eloquent way to do this... but it gets the job done :)
ArrayXi MaskToRowId(ArrayXb const& mask) {
    std::vector<int> row_ids;
    row_ids.reserve(mask.rows());

    for (Eigen::Index i{0}; i < mask.rows(); i++) {
        if (mask(i) == true) {
            row_ids.push_back(i);
        }
    }

    return ToEigen(row_ids);
}

ArrayXi ToEigen(std::vector<int> const& vector) { return Eigen::Map<ArrayXi const>(vector.data(), std::size(vector)); }

}  // namespace reprojection::eigen_utilities