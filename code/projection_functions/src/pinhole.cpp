#include "projection_functions/pinhole.hpp"

#include "eigen_utilities/camera.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): This function seems like it is missing some understanding of the fundamental problem we are facing, but
// this is just the start :)
Eigen::MatrixX2d PinholeProjection(Eigen::Matrix3d const& K, Eigen::MatrixX3d points) {
    Eigen::Array<double, 4, 1> const K_star{eigen_utilities::FromK(K)};

    Eigen::MatrixX2d pixels(points.rows(), 2);
    for (int i{0}; i < points.rows(); ++i) {
        pixels.row(i) = PinholeProjection<double>(K_star.data(), points.row(i).leftCols(3));
    }

    return pixels;
}

}  // namespace reprojection::projection_functions