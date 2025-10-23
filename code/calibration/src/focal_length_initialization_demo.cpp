#include "calibration/focal_length_initialization_demo.hpp"

#include <numeric>

#include "eigen_utilities/grid.hpp"
#include "focal_length_initialization.hpp"

namespace reprojection::calibration {

double InitializeFocalLengthFromTarget(Eigen::MatrixX2d const& pixels, Eigen::ArrayX2i const& indices) {
    assert(pixels.rows() == indices.rows());

    // WARN(Jack): The following logic assumes that the target row/col indexes will always start with zero! For multi
    // target settings this might not always be the case. But if multi-target setups are later supported we need to work
    // on a couple places to make it work, so we ingore this possibility for now :)
    Eigen::Array2i const target_row_col{indices.colwise().maxCoeff()};

    std::vector<double> estimated_focal_lengths;
    estimated_focal_lengths.reserve(target_row_col.prod());
    for (int i{0}; i <= target_row_col(0); ++i) {
        Eigen::ArrayXi const row_i_mask{MaskTargetIndicesDimension(indices, i, Dimension::Row)};
        Eigen::MatrixX2d const row_i_pixels{pixels(row_i_mask, Eigen::all)};

        // TODO(Jack): What is the appropriate minumum number of points here? The circle fitting technically works even
        // with two points, but if that is accurate I do not know.
        if (row_i_pixels.rows() < 3) {
            continue;
        }

        for (int j{0}; j <= target_row_col(1); ++j) {
            Eigen::ArrayXi const col_i_mask{MaskTargetIndicesDimension(indices, j, Dimension::Col)};
            Eigen::MatrixX2d const col_i_pixels{pixels(col_i_mask, Eigen::all)};

            if (col_i_pixels.rows() < 3) {
                continue;
            }

            auto const f{EstimateFocalLength(row_i_pixels, col_i_pixels)};
            if (f.has_value()) {
                estimated_focal_lengths.push_back(f.value());
            }
        }
    }

    // TODO(Jack): This is an extremely naive way to do this! There must be some better strategy to handle outliers etc.
    // TODO(Jack): Handle the case where there is no succesfull focal length estimation (i.e. return optional).
    double const sum_f{std::accumulate(std::cbegin(estimated_focal_lengths), std::cend(estimated_focal_lengths), 0.0)};
    double const mean_f{sum_f / std::size(estimated_focal_lengths)};

    return mean_f;
}
}

// THERE IS A PRETTY SIMILAR FUNCTION IN eigen_utilities (MaskIndices), FIGURE OUT HOW TO NOT COPY AND PASTE
Eigen::ArrayXi MaskTargetIndicesDimension(Eigen::ArrayX2i const& indices, int const id, Dimension const dimension) {
    std::vector<int> mask;
    mask.reserve(indices.rows());

    for (Eigen::Index i{0}; i < indices.rows(); i++) {
        if (indices.row(i)(static_cast<int>(dimension)) == id) {
            mask.push_back(i);
        }
    }

    return eigen_utilities::ToEigen(mask);
}

}  // namespace reprojection::calibration