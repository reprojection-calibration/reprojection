#pragma once

#include <Eigen/Dense>

namespace reprojection::eigen_utilities {

// Requesting even_only has the effect of allowing us to produce asymmetric grids like those required by the asymmetric
// circle grid target.
Eigen::ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only = false);

Eigen::ArrayXi MaskIndices(Eigen::ArrayXi const& array);

Eigen::ArrayXi ToEigen(std::vector<int> const& vector);

}  // namespace reprojection::eigen_utilities