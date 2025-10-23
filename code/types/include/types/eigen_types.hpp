#pragma once

#include <Eigen/Dense>

// TODO(Jack): Should we really put this into the ::types sub-namespace? Maybe these are so universal they should be
// directly available in the reprojection namespace directly
namespace reprojection::types {

using M
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
    Eigen::ArrayX2i indices;
};

}  // namespace reprojection::types