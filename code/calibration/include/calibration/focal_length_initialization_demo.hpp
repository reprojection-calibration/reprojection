#pragma once

#include <Eigen/Dense>

namespace reprojection::calibration {

enum class Dimension { Row = 0, Col = 1 };

// THERE IS A PRETTY SIMILAR FUNCTION IN eigen_utilities (MaskIndices), FIGURE OUT HOW TO NOT COPY AND PASTE
Eigen::ArrayXi MaskTargetIndicesDimension(Eigen::ArrayX2i const& indices, int const id, Dimension const dimension);

double InitializeFocalLengthFromTarget(Eigen::MatrixX2d const& pixels, Eigen::ArrayX2i const& indices);

}  // namespace reprojection::calibration