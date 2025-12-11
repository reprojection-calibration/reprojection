#pragma once

#include <tuple>

#include "types/eigen_types.hpp"

namespace reprojection::pnp {

// Adopted from
// https://stackoverflow.com/questions/46110917/eigen-replicate-items-along-one-dimension-without-useless-allocations?noredirect=1&lq=1
//
// Given matrix:
//      A = 0, 1,
//          2, 3,
//          4, 5
//
// Return matrix:
//      B = 0, 1,
//          0, 1,
//          2, 3,
//          2, 3,
//          4, 5
//          4, 5
MatrixXd InterleaveRowWise(MatrixXd const& matrix);

// This function was designed with the normalization required for the DLT in mind! This means that after normalization
// we want the "average distance of a point x from the origin is equal to sqrt(n)" (where n is the dimension of the
// non-homogeneous point). See MVG 4.4.4 heading "Isotropic Scaling"
std::tuple<MatrixXd, MatrixXd> NormalizeColumnWise(MatrixXd const& matrix);

Isometry3d ToIsometry3d(MatrixX3d const& R, Vector3d const& T);

}  // namespace reprojection::pnp