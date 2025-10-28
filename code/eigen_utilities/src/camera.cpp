#include "eigen_utilities/camera.hpp"

namespace reprojection::eigen_utilities {

Eigen::Array<double, 4, 1> FromK(Matrix3d const& matrix) {
    return {matrix(0, 0), matrix(1, 1), matrix(0, 2), matrix(1, 2)};
};

}  // namespace reprojection::eigen_utilities