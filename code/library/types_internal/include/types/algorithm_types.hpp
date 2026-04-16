#pragma once

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
    Bundle bundle;
    ArrayX2i indices;
};

struct ImuData {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

// TODO(Jack): Where does this belong? It does not really fit neatly into nay category of types because it really just
// has to do with data input and output. Except for feature extraction it is not used anywhere.
struct EncodedImage {
    std::vector<uchar> data;
};

}  // namespace reprojection