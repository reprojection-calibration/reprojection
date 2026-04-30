#include "eigen_utilities/camera.hpp"

namespace reprojection::eigen_utilities {

Array3d FromK(Matrix3d const& matrix) {
    // Take the average of the two focal lengths as our convention is to only use one single one in the library.
    double const f{(matrix(0, 0) + matrix(1, 1)) / 2};

    return {f, matrix(0, 2), matrix(1, 2)};
};

}  // namespace reprojection::eigen_utilities