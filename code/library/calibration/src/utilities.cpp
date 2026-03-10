
#include "utilities.hpp"

#include <set>

#include "eigen_utilities/grid.hpp"

namespace reprojection::calibration {

std::pair<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target) {
    auto const rows{ExtractBundlesByDimension(target, Dimension::Row)};
    auto const cols{ExtractBundlesByDimension(target, Dimension::Col)};

    return {rows, cols};
}

// See https://stackoverflow.com/questions/14740867/find-unique-numbers-in-array
std::vector<Bundle> ExtractBundlesByDimension(ExtractedTarget const& target, Dimension dim) {
    auto const column{target.indices.col(static_cast<int>(dim))};
    std::set<int> const ids{std::cbegin(column), std::cend(column)};

    std::vector<Bundle> bundles;
    for (auto const id : ids) {
        ArrayXb const mask{(column.unaryExpr([id](int const x) { return x == id; }) == true)};

        ArrayXi const indices{eigen_utilities::MaskToRowId(mask)};
        bundles.push_back(target.bundle(indices));
    }

    return bundles;
}

}  // namespace reprojection::calibration