#include "calibration/initialize_focal_length.hpp"

#include <set>

#include "eigen_utilities/grid.hpp"

namespace reprojection::calibration {

std::tuple<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target) {
    // Get rows
    // See https://stackoverflow.com/questions/14740867/find-unique-numbers-in-array
    std::set<int> const row_ids{target.indices.col(0).cbegin(), target.indices.col(0).cend()};
    std::vector<Bundle> rows;
    for (auto const id : row_ids) {
        ArrayXi const row_mask{
            (target.indices.col(0).unaryExpr([id](int const x) { return x == id; }) == true).cast<int>()};
        ArrayXi const indices{eigen_utilities::MaskIndices(row_mask)};

        rows.push_back(target.bundle(indices));
    }

    // TODO(Jack): Can we avoid the copy and paste?
    // Get cols
    std::set<int> const col_ids{target.indices.col(1).cbegin(), target.indices.col(1).cend()};
    std::vector<Bundle> cols;
    for (auto const id : col_ids) {
        ArrayXi const col_mask{
            (target.indices.col(1).unaryExpr([id](int const x) { return x == id; }) == true).cast<int>()};
        ArrayXi const indices{eigen_utilities::MaskIndices(col_mask)};

        cols.push_back(target.bundle(indices));
    }

    return {rows, cols};
}

}  // namespace reprojection::calibration