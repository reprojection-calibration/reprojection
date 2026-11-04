#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::eigen_utilities {

// TODO(Jack): This logic is pretty copy and pasted from equivalent code in the feature_extraction package utilities. I
// removed the asymmetric grid part here because we no longer needed that for our current "left over" applications we
// have here. Maybe one day we can unify these, but it's really not important.
ArrayX2i GenerateGridIndices(int rows, int cols);

// TODO(Jack): I think this belongs somewhere that is more focused on masking than the grid.
ArrayXi MaskToRowId(ArrayXb const& mask);

ArrayXi ToEigen(std::vector<int> const& vector);

}  // namespace reprojection::eigen_utilities