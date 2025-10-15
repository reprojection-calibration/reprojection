#include "matrix_utilities.hpp"

#include <iostream>

namespace reprojection_calibration::pnp {

Eigen::MatrixXd InterleaveRowWise(Eigen::MatrixXd const& matrix) {
    Eigen::Index const n_rows{matrix.rows()};
    Eigen::MatrixXd const interleaved_matrix{matrix(Eigen::ArrayXi::LinSpaced(2 * n_rows, 0, n_rows), Eigen::all)};

    return interleaved_matrix;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> NormalizeColumnWise(Eigen::MatrixXd const& matrix) {
    Eigen::VectorXd const center{matrix.colwise().mean()};
    double const mean_magnitude{((matrix.rowwise() - center.transpose())).rowwise().norm().mean()};

    Eigen::Index const n{matrix.cols()};
    double const scale{std::sqrt(n) / mean_magnitude};

    // TODO(Jack): Where is the source or link for this code? If it exists please link it because this is hard to
    // understand in the future.
    Eigen::MatrixXd Tf{Eigen::MatrixXd::Identity(n + 1, n + 1)};
    Tf.topRightCorner(n, 1) = -scale * center;
    Tf.diagonal().topRows(n) *= scale;

    // TODO(Jack): Make a function that handles the transpose magic, or consider doing these in a different order. Just
    // be consistent regardless :)
    Eigen::MatrixXd const normalized_matrix{(Tf * (matrix.rowwise().homogeneous()).transpose()).transpose()};

    return {normalized_matrix.leftCols(n), Tf};
}

Eigen::Isometry3d ToIsometry3d(Eigen::MatrixX3d const& R, Eigen::Vector3d const& T) {
    Eigen::Isometry3d tf;
    tf.linear() = R;
    tf.translation() = T;

    return tf;
}

}  // namespace reprojection_calibration::pnp