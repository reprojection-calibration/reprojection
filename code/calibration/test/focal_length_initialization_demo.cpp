
#include <gtest/gtest.h>

#include <numeric>

#include "eigen_utilities/grid.hpp"
#include "focal_length_initialization.hpp"

using namespace reprojection;
using namespace reprojection::calibration;

// NOTE(Jack): This demo will count on the fact that the points, ids, pixels are correspondent!

enum class Dimension { Row = 0, Col = 1 };

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

TEST(XXX, FFFFF) {
    // NOTE(Jack): We will use this to simulate the mythical non-existent feature frame where one row and one column
    // wrap completely around in a circle! This is not realistic, but allows us to figure out the basic logic.
    // First four pixels are (x-1)^2 + (y-1)^2 = 1 and the last four are // (x-2)^2 + (y-2)^2 = 1
    Eigen::MatrixX2d const pixels{{0, 1}, {2, 1}, {1, 0}, {1, 2}, {1, 2}, {3, 2}, {2, 1}, {2, 3}};
    Eigen::MatrixX3d const points(pixels.rows(), 3);  // Empty as they are not used
    // Because the two sets of pixels are totally unrelated we cannot have them share any rows or columns, therefore the
    // second circle starts at row 1 and is in column 4 - no shared rows or columns at all!
    Eigen::ArrayX2i const indices{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 4}, {2, 4}, {3, 4}, {4, 4}};

    double const f{InitializeFocalLengthFromTarget(pixels, indices)};

    EXPECT_FLOAT_EQ(f, 0.45015815);
}

TEST(XXX, FFFFFCCC) {
    Eigen::ArrayX2i const indices{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 4}, {2, 4}, {3, 4}, {4, 4}};

    Eigen::ArrayXi const row_0_mask{MaskTargetIndicesDimension(indices, 0, Dimension::Row)};
    EXPECT_TRUE(row_0_mask.isApprox(Eigen::Array<int, 4, 1>{0, 1, 2, 3}));

    Eigen::ArrayXi const col_0_mask{MaskTargetIndicesDimension(indices, 4, Dimension::Col)};
    EXPECT_TRUE(col_0_mask.isApprox(Eigen::Array<int, 4, 1>{4, 5, 6, 7}));
}