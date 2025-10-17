#include "homography_decomposition.hpp"

#include "dlt_solving.hpp"
#include "geometry/lie.hpp"
#include "matrix_utilities.hpp"

namespace reprojection::pnp {

// Assumes that the pixels are in normalized ideal image space
Eigen::Isometry3d FullPipeline(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    auto const [t,
                R]{FindHomography(pixels, points(Eigen::all, {0, 1}))};  // CUTS OFF THE Z DIMENSION NO MATTER WHAT!!!

    return ToIsometry3d(R, t);
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> FindHomography(Eigen::MatrixX2d const& points_src,
                                                            Eigen::MatrixX2d const& points_dst) {
    auto const A{ConstructA<3>(points_src, points_dst)};
    auto H{SolveForP<3>(A)};

    // Reference https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
    H /= H.col(0).norm();  // First column magnitude 1 - constrains scale

    Eigen::Vector3d const h_norms{H.colwise().norm()};

    H.col(0) = H.col(0) / h_norms(0);
    H.col(1) = H.col(1) / h_norms(1);
    Eigen::Vector3d const t{H.col(2) * (2.0 / (h_norms(0) + h_norms(1)))};
    H.col(2) = H.col(0).cross(H.col(1));

    // Introduces error but makes it a real rotation matrix !!!!!
    Eigen::Matrix3d const cleaned_H{reprojection::geometry::Exp((reprojection::geometry::Log(H)))};

    return {t, cleaned_H};
}

// CURRENTLY UNUSED - code now always assumes points have z=0
std::tuple<Eigen::MatrixX2d, Eigen::Isometry3d> NormalizePointsForHomographySolving(Eigen::MatrixX3d const& points) {
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

    return {normalized_chopped_points, T};
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