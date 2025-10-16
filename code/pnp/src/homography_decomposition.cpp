#include "homography_decomposition.hpp"

#include "matrix_utilities.hpp"

namespace reprojection::pnp {

Eigen::MatrixX2d NormalizePointsForHomographySolving(Eigen::MatrixX3d const& points) {
    auto const [_, V]{WhatDoWeNameThis(points)};

    Eigen::Matrix3d R{V};
    if (V(0, 2) * V(0, 2) + V(1, 2) * V(1, 2) < 1e-10) {
        R = Eigen::Matrix3d::Identity();
    } else if (R.determinant() < 0) {
        R *= -1;
    }

    Eigen::Vector3d const center{points.colwise().mean()};
    Eigen::Vector3d const t{-R * center};
    Eigen::Isometry3d const T{ToIsometry3d(R, t)};

    Eigen::MatrixX2d const normalized_chopped_points{
        (T * points.rowwise().homogeneous().transpose()).transpose()(Eigen::all, {0, 1})};

    return normalized_chopped_points;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> WhatDoWeNameThis(Eigen::MatrixX3d const& points) {
    Eigen::Vector3d const center{points.colwise().mean()};
    Eigen::Matrix3d const covariance{(points.rowwise() - center.transpose()).transpose() *
                                     (points.rowwise() - center.transpose())};

    // TOOD(Jack): Are these reasonable svd computation options?
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);

    return {svd.singularValues(), svd.matrixV()};
}

bool IsPlane(Eigen::MatrixX3d const& points) {
    auto const [singular_values, _]{WhatDoWeNameThis(points)};

    if (singular_values[2] / singular_values[1] < 1e-3) {
        return true;
    }

    return false;
}

}  // namespace reprojection::pnp