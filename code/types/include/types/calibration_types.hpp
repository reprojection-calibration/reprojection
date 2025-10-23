#pragma once

#include "types/eigen_types.hpp"

namespace reprojection {

struct FeatureFrame {
    MatrixX2d pixels;
    MatrixX3d points;
    ArrayX2i indices;
};

}  // namespace reprojection