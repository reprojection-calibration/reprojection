#pragma once

#include <vector>

#include "types/algorithm_types.hpp"

namespace reprojection::calibration {

enum class Dimension { Row = 0, Col = 1 };

std::pair<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target);

std::vector<Bundle> ExtractBundlesByDimension(ExtractedTarget const& target, Dimension dim);

}  // namespace reprojection::calibration