#include "../include/pnp/dlt.hpp"

#include "dlt_matrix_decompositions.hpp"
#include "dlt_solving.hpp"
#include "eigen_utilities/camera.hpp"

namespace reprojection::pnp {

// NOTE(Jack): The number of pixels and points has to match! However, because Dlt is part of the internal API, and the
// number of correspondences is already check in the public facing interface, we do not check it again here.
// NOTE(Jack): We probably mainly want only the pose, but we calculate K anyway as part of the process, so following the
// "law of useful return", we return K too.
std::tuple<Isometry3d, Array4d> Dlt23(Bundle const& bundle) {
    auto const [normalized_pixels, tf_pixels]{NormalizeColumnWise(bundle.pixels)};
    auto const [normalized_points, tf_points]{NormalizeColumnWise(bundle.points)};

    Eigen::Matrix<double, Eigen::Dynamic, 12> const A{ConstructA<4>(normalized_pixels, normalized_points)};
    Eigen::Matrix<double, 3, 4> const P{SolveForH<4>(A)};
    Eigen::Matrix<double, 3, 4> const P_star{tf_pixels.inverse() * P * tf_points};  //  Denormalize

    // Extract camera parameters
    auto const [K, R]{DecomposeMIntoKr(P_star.leftCols(3))};
    Vector3d const t{CalculateCameraCenter(P_star)};

    return {ToIsometry3d(R, -R * t), eigen_utilities::FromK(K)};
}

// WARN(Jack): Assumes that pixel coordinates are normalized ideal image coordinates, not pixel values.
Isometry3d Dlt22(Bundle const& bundle) {
    // ERROR(Jack): Always assumes we aligned with the Z dimension as the plane! CUTS OFF THE Z DIMENSION NO MATTER
    // WHAT!!!
    MatrixX2d const chopped_points{bundle.points(Eigen::all, {0, 1})};
    auto const [normalized_points, tf_points]{NormalizeColumnWise(chopped_points)};

    // WARN(Jack): If we had exactly four correspondences, this means that A would be 8x9. For the svd that follows is
    // that ok? Do we need to zero pad anything or simply check that we have at least five points? Or is it no problem
    // at all?
    auto const A{ConstructA<3>(bundle.pixels, normalized_points)};
    Matrix3d const H{SolveForH<3>(A)};
    Matrix3d const H_star{H * tf_points};  //  Denormalize

    auto const [R, t]{DecomposeHIntoRt(H_star)};

    return ToIsometry3d(R, t);
}

}  // namespace reprojection::pnp