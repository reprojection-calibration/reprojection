#include "matrix_utilities.hpp"

namespace reprojection::pnp {

MatrixXd InterleaveRowWise(MatrixXd const& mat) {
    Eigen::Index const n_rows{mat.rows()};
    MatrixXd const interleaved_matrix{mat(ArrayXi::LinSpaced(2 * n_rows, 0, n_rows), Eigen::all)};

    return interleaved_matrix;
}

std::tuple<MatrixXd, MatrixXd> NormalizeColumnWise(MatrixXd const& mat) {
    VectorXd const center{mat.colwise().mean()};
    double const mean_magnitude{((mat.rowwise() - center.transpose())).rowwise().norm().mean()};

    Eigen::Index const n{mat.cols()};
    double const scale{std::sqrt(n) / mean_magnitude};

    // TODO(Jack): Where is the source or link for this code? If it exists please link it because this is hard to
    //  understand in the future.
    MatrixXd tf{MatrixXd::Identity(n + 1, n + 1)};
    tf.topRightCorner(n, 1) = -scale * center;
    tf.diagonal().topRows(n) *= scale;

    // TODO(Jack): Make a function that handles the transpose magic, or consider doing these in a different order. Just
    //  be consistent regardless :)
    MatrixXd const normalized_matrix{(tf * (mat.rowwise().homogeneous()).transpose()).transpose()};

    return {normalized_matrix.leftCols(n), tf};
}

Isometry3d ToIsometry3d(MatrixX3d const& R, Vector3d const& t) {
    Isometry3d tf;
    tf.linear() = R;
    tf.translation() = t;

    return tf;
}

}  // namespace reprojection::pnp