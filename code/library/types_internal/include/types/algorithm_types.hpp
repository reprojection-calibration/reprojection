#pragma once

#include <vector>

#include "types/eigen_types.hpp"

namespace reprojection {

struct Bundle {
    Bundle operator()(ArrayXi const& valid_ids) const {
        return Bundle{pixels(valid_ids, Eigen::all), points(valid_ids, Eigen::all)};
    }

    MatrixX2d pixels;
    MatrixX3d points;
};

struct ExtractedTarget {
    ExtractedTarget operator()(ArrayXi const& valid_ids) const {
        return ExtractedTarget{bundle(valid_ids), indices(valid_ids, Eigen::all)};
    }

    Bundle bundle;
    ArrayX2i indices;
};

struct ImuData {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

struct ImageBuffer {
    std::vector<unsigned char> data;
};

}  // namespace reprojection