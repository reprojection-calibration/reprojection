#pragma once

#include <Eigen/Dense>

namespace reprojection {

struct FeatureFrame {
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
    Eigen::ArrayX2i indices;
};

}  // namespace reprojection