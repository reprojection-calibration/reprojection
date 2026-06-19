#include "cubic_spline_c3_init.hpp"

#include <Eigen/SparseCholesky>
#include <ranges>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/spline_initialization.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

#include "sparse_utilities.hpp"

namespace reprojection::spline {

std::pair<MatrixNXd, TimeHandler> InitializeC3SplineState(PositionMeasurements const& measurements,
                                                          size_t const num_segments) {
    // WARN(Jack): We might have some rounding error here due calculating delta_t_ns, at this time that is no known
    // problem.
    uint64_t const t0_ns{std::cbegin(measurements)->first};
    uint64_t const tn_ns{std::crbegin(measurements)->first};  // Reverse iterator ("rbegin")!
    uint64_t const delta_t_ns{(tn_ns - t0_ns) / num_segments};
    TimeHandler const time_handler{t0_ns, delta_t_ns};

    auto const [A, b]{CubicBSplineC3Init::BuildAb(measurements, num_segments, time_handler)};

    // NOTE(Jack): At this time lambda here is hardcoded, it might make sense at some time in the future to parameterize
    // this, but currently I see no scenario where we can really expect the user to parameterize it, so we leave it
    // hardcoded for now.
    // NOTE(Jack): The lambda that you need to use is very large, about e7/e8/e9 magnitude because we use nanoseconds
    // timestamps which results in very small values in the omega matrix otherwise.
    CoefficientBlock const omega{BuildOmega(delta_t_ns, 1e12)};
    Eigen::SparseMatrix<double> const Q{DiagonalSparseMatrix(omega, N, num_segments)};

    // NOTE(Jack): When we first tried to apply this to larger spline initialization problems (ex. 2000 segments) it was
    // slow as hell and took about 55 seconds on my laptop to initialize the rotation and translation. But then I used a
    // sparse solver and it cut the time down to about 600ms. Therefore I think we are on the right track here using
    // sparse logic.
    // WARN(Jack): In the documentation for the .sparseView() method it says "This method is typically used when
    // prototyping to convert a quickly assembled dense Matrix D to a SparseMatrix S". That sentence implies it should
    // only be used for prototyping, but it solved my problem (initialization time cut from 55s to 600ms) so honestly I
    // am asking myself why I should refactor the entire initialization code to be "sparse by default" and not use dense
    // matrices like we do during the problem construction. Maybe it would be nice to transfer all the code directly to
    // work on the sparse representation, but at this time I see no benefit.
    // TODO(Jack): We should actually build A as a sparse matrix so we can save space and avoid having all these manual
    //  sparse constructions/sparse views.
    Eigen::SparseMatrix<double> const A_n{
        Eigen::SparseMatrix<double>(A.sparseView().transpose()) * Eigen::SparseMatrix<double>(A.sparseView()) + Q};
    MatrixXd const b_n{A.transpose().sparseView() * b};

    // See the section "Sparse solver concept" in
    // https://libeigen.gitlab.io/eigen/docs-nightly/group__TopicSparseSystems.html
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(A_n);
    // TODO(Jack): We should refactor this entire init function to return optional!
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Failed: solver.compute(A_n.sparseView());");  // LCOV_EXCL_LINE
    }

    VectorXd const x{solver.solve(b_n)};
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Failed: solver.solve(b_n);");  // LCOV_EXCL_LINE
    }

    // TODO(Jack): Is there a better way to calculate the number of control points here than x.rows()/N?
    return {Eigen::Map<MatrixNXd const>(x.data(), N, x.rows() / N), time_handler};
}

std::pair<MatrixXd, VectorXd> CubicBSplineC3Init::BuildAb(PositionMeasurements const& positions,
                                                          size_t const num_segments, TimeHandler const& time_handler) {
    // NOTE(Jack): For both measurement_dim and control_point_dim we are talking about the "vectorized" dimensions.
    // This means how many values are there when we stack all the individual vectors (i.e. measurements or
    // control points) into one big vector to be used in the Ax=b problem. There x is the control points vector of
    // length control_point_dim and b is the measurement vector of length measurement_dim.
    size_t const measurement_dim{std::size(positions) * N};
    size_t const num_control_points{num_segments + D};
    size_t const control_point_dim{num_control_points * N};

    MatrixXd A{MatrixXd::Zero(measurement_dim, control_point_dim)};

    for (size_t j{0}; auto timestamp_ns : positions | std::views::keys) {
        // ERROR(Jack): HACK - At this time we have no principled strategy to deal with the end conditions of the
        // spline, therefore we need this hack here. What this hack does is ensure that at the very end of the spline,
        // when time_ns_i is at the end (corresponds to the last measurement), it does not start another time segment
        // past the end of the spline, but instead stays on the last valid segment at the very end (ex. u_i=0.99999).
        // This is definitely a hack, but it "works"!
        if (j == std::size(positions) - 1) {
            timestamp_ns -= static_cast<std::uint64_t>(1 + 0.01 * time_handler.delta_t_ns_);
        }

        // WARN(Jack): Unprotected optional access! Technically we should always been in a valid time segment because
        // the measurement times are always non-decreasing and define the limit of the valid times themselves. In
        // combination with the hack described above, we should not get problems here. However, in reality this shows
        // that maybe we are not describing or capturing the problem well. A better solution here is welcome!
        auto const [u_i, i]{time_handler.SplinePosition(timestamp_ns, num_control_points).value()};
        A.block(j * N, i * N, N, KxN) = BlockifyWeights(u_i);

        j += 1;
    }

    VectorXd b{VectorXd{measurement_dim, 1}};

    for (size_t i{0}; auto const& position_i : positions | std::views::values) {
        b.segment(i * N, N) = position_i.position;

        i += 1;
    }

    return {A, b};
}

CubicBSplineC3Init::ControlPointBlock CubicBSplineC3Init::BlockifyWeights(double const u_i) {
    // WARN(Jack): We use R3Spline::B<> even for the so3 spline interpolation! This is somehow inconsistent because the
    // so3 spline is a cumulative spline and has a different basis matrix than the R3 spline. During testing of the
    // interpolation on the spline trajectory it seemed to work regardless, but maybe there is an error here anyway that
    // will come out in edge cases!
    VectorKd const weights_i{R3Spline::B<DerivativeOrder::Null>(u_i)};

    ControlPointBlock sparse_weights{ControlPointBlock::Zero()};
    for (int i{0}; i < K; ++i) {
        sparse_weights.block(0, i * N, N, N) = Matrix3d::Identity() * weights_i[i];
    }

    return sparse_weights;
}

CoefficientBlock BlockifyBlendingMatrix(MatrixKd const& blending_matrix) {
    auto build_block = [](Vector4d const& element) {
        Eigen::Matrix<double, KxN, N> X{Eigen::Matrix<double, KxN, N>::Zero()};
        for (int i = 0; i < N; i++) {
            X.block(i * K, i, K, 1) = element;
        }

        return X;
    };

    CoefficientBlock M{CoefficientBlock::Zero()};
    for (int i{0}; i < K; ++i) {
        M.block(0, i * N, KxN, N) = build_block(blending_matrix.row(i));
    }

    return M;
}  // LCOV_EXCL_LINE

MatrixXd DerivativeOperator(int const order) {
    MatrixXd D{MatrixXd::Zero(order, order)};
    // TODO(Jack): Why is this hardcoded to DerivativeOrder::First here?
    D.diagonal(1) = PolynomialCoefficients(order).row(static_cast<int>(DerivativeOrder::First)).rightCols(order - 1);

    return D;
}  // LCOV_EXCL_LINE

MatrixXd HilbertMatrix(int const size) {
    // Hilbert matrix coefficients are as follows: 1/1, 1/2, 1/3,..., 1/size
    VectorXd const hilbert_coefficients{Eigen::VectorXd::LinSpaced(size, 1, size).cwiseInverse()};

    return HankelMatrix(hilbert_coefficients);
}

MatrixXd HankelMatrix(VectorXd const& coefficients) {
    assert(coefficients.size() % 2 == 1);  // Only allowed odd number of coefficients (only square matrices!)

    Eigen::Index const size{(coefficients.size() + 1) / 2};
    MatrixXd hankel{MatrixXd(size, size)};
    for (int row{0}; row < size; ++row) {
        for (int col{0}; col < size; ++col) {
            hankel(row, col) = coefficients(row + col);
        }
    }

    return hankel;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::spline
