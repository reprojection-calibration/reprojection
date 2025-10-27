#pragma once

#include "matrix_utilities.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::pnp {

// NOTE(Jack): We know the K matrix ahead of time, at least that is then assumption the opencv pnp implementation
// makes, therefore the question is; Can we somehow use that knowledge to make our DLT better, and eliminate that we
// solve for K here? Or should we just follow the law of useful return and return T and K? For now we will do the
// latter but let's keep our eyes peeled for possible simplifications in the future.
std::tuple<Isometry3d, Array4d> Dlt23(MatrixX2d const& pixels, MatrixX3d const& points);

Isometry3d Dlt22(MatrixX2d const& pixels, MatrixX3d const& points);

}  // namespace reprojection::pnp