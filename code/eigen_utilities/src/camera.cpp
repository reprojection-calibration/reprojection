#include "eigen_utilities/camera.hpp"

namespace reprojection::eigen_utilities {

Matrix3d ToK(Eigen::Array<double, 4, 1> const& array) {
    Matrix3d K{Matrix3d::Identity()};
    K(0, 0) = array[0];
    K(1, 1) = array[1];
    K(0, 2) = array[2];
    K(1, 2) = array[3];

    return K;
};

Eigen::Array<double, 4, 1> FromK(Matrix3d const& matrix) {
    return {matrix(0, 0), matrix(1, 1), matrix(0, 2), matrix(1, 2)};
};

}  // namespace reprojection::eigen_utilities