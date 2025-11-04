#pragma once

#include <tuple>

#include "types/eigen_types.hpp"

namespace reprojection::pnp {

// Inspired by https://www.physicsforums.com/threads/rq-decomposition-from-qr-decomposition.261739/
// We implement RQ decomposition in terms of Eigen's built in QR decomposition
std::tuple<Matrix3d, Matrix3d> RqDecomposition(Matrix3d const& matrix);

// NOTE(Jack): MVG section "6.2.4 Decomposition of the camera matrix" refers to the first three columns of the camera
// matrix P as M.
// Adopted from https://ksimek.github.io/2012/08/14/decompose/
std::tuple<Matrix3d, Matrix3d> DecomposeMIntoKr(Matrix3d const& M);

// NOTE(Jack): MVG section "6.2.4 Finding the camera center"
// We automatically return the non-homogenous value, maybe there is a reason to return it as a homogeneous point but I
// do not see it just yet.
Vector3d CalculateCameraCenter(Eigen::Matrix<double, 3, 4> const& P);

// NOTE(Jack): Reference https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html - the material is taken from this
// slide deck on web.archive.org
// https://web.archive.org/web/20171226115739/https://ags.cs.uni-kl.de/fileadmin/inf_ags/3dcv-ws11-12/3DCV_WS11-12_lec04.pdf
std::tuple<Matrix3d, Vector3d> DecomposeHIntoRt(Matrix3d const& H);

}  // namespace reprojection::pnp