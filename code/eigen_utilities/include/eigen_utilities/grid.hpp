#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// Requesting even_only has the effect of allowing us to produce asymmetric grids like those required by the asymmetric
// circle grid target.
ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only = false);

ArrayXi MaskIndices(ArrayXi const& array);

ArrayXi ToEigen(std::vector<int> const& vector);

}  // namespace reprojection::eigen_utilities