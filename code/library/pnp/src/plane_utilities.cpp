#include "plane_utilities.hpp"

namespace reprojection::pnp {

// WARN(Jack): this function does no error handling! Importantly it will classify collinear points as being on a plane,
// when I think that is actually an error case or edge condition that should be specially handled to prevent division by
// zero here. Also this does not check that the points matrix has the minumum required number of points!
bool IsPlane(MatrixX3d const& points) {
    auto const [singular_values, _]{Pca(points)};

    if (singular_values[2] / singular_values[1] < 1e-3) {
        return true;
    }

    return false;
}

std::tuple<Vector3d, Matrix3d> Pca(MatrixX3d const& points) {
    Vector3d const center{points.colwise().mean()};
    Matrix3d const covariance{(points.rowwise() - center.transpose()).transpose() *
                              (points.rowwise() - center.transpose())};

    // TOOD(Jack): Are these reasonable svd computation options?
    Eigen::JacobiSVD<MatrixXd> svd;
    svd.compute(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);

    return {svd.singularValues(), svd.matrixV()};
}

}  // namespace reprojection::pnp