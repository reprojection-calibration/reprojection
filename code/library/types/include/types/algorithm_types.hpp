#pragma once

#include "types/eigen_types.hpp"

namespace reprojection {

struct Bundle {
    Bundle operator()(ArrayXi const& indices) const {
        return Bundle{pixels(indices, Eigen::all), points(indices, Eigen::all)};
    }

    MatrixX2d pixels;
    MatrixX3d points;
};

struct ExtractedTarget {
    Bundle bundle;
    ArrayX2i indices;
};

struct ImuData {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

}  // namespace reprojection