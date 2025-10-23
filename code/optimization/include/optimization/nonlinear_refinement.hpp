#pragma once

#include "types/eigen_types.hpp"

namespace  reprojection::optimization {



std::tuple<Isometry3d, Matrix3d> NonlinearRefinement(MatrixX2d const& pixels, MatrixX3d const& points,
                                                     Isometry3d const& initial_pose, Matrix3d const& initial_K);

}  // namespace  reprojection::optimization
