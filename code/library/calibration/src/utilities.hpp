#pragma once

#include <vector>

#include "types/algorithm_types.hpp"

namespace reprojection::calibration {

std::pair<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target);

}  // namespace reprojection::calibration