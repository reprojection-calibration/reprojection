#include "dlt_solving.hpp"
#include "matrix_utilities.hpp"
#include "plane_utilities.hpp"

namespace reprojection::pnp {

bool IsPlane(Eigen::MatrixX3d const& points) {
    auto const [singular_values, _]{Pca(points)};

    if (singular_values[2] / singular_values[1] < 1e-3) {
        return true;
    }

    return false;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> Pca(Eigen::MatrixX3d const& points) {
    Eigen::Vector3d const center{points.colwise().mean()};
    Eigen::Matrix3d const covariance{(points.rowwise() - center.transpose()).transpose() *
                                     (points.rowwise() - center.transpose())};

    // TOOD(Jack): Are these reasonable svd computation options?
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);

    return {svd.singularValues(), svd.matrixV()};
}

}  // namespace reprojection::pnp