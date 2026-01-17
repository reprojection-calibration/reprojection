#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// Requesting even_only=true has the effect of allowing us to produce asymmetric grids like those required by the
// asymmetric circle grid target.
ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only = false);

ArrayXi MaskToRowId(ArrayXb const& mask);
ArrayXi AntiMaskToRowId(ArrayXb const& mask);

ArrayXi ToEigen(std::vector<int> const& vector);

}  // namespace reprojection::eigen_utilities