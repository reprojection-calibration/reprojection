#include "projection_functions/pinhole.hpp"

#include "eigen_utilities/camera.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): This is a kind of helper function so that when we are working with eigen and just need a quick projection
// we can get it. What the long term prospects for a concise clear camera projection interface are still unclear.
Eigen::MatrixX2d Pinhole::Project(Array4d const& intrinsics, Eigen::MatrixX3d points) {
    Eigen::MatrixX2d pixels(points.rows(), 2);
    for (int i{0}; i < points.rows(); ++i) {
        pixels.row(i) = Project<double>(intrinsics, points.row(i).leftCols(3));
    }

    return pixels;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::projection_functions