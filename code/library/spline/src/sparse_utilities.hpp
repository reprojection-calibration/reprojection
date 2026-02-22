#pragma once

#include <Eigen/SparseCore>

#include "types/eigen_types.hpp"

namespace reprojection::spline {

// See section "Filling a sparse matrix" - https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialSparse.html
Eigen::SparseMatrix<double> DiagonalSparseMatrix(MatrixXd const& block, size_t const stride, size_t const count);

}  // namespace reprojection::spline
