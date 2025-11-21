#include "spline/spline_initialization.hpp"

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

CubicBSplineC3 CubicBSplineC3Init::InitializeSpline(std::vector<C3Measurement> const& measurements,
                                                    size_t const num_segments) {
    // TODO(Jack): Will rounding effect the time handling here?
    // TODO(Jack): Given a certain number of measurement is there a limit/boundary to valid num_segments?
    CubicBSplineC3 spline{measurements[0].t_ns, (measurements.back().t_ns - measurements.front().t_ns) / num_segments};
    auto const [A, b]{BuildAb(measurements, num_segments, spline.time_handler)};

    // TODO(Jack): Pass lambda as parameter
    MatrixXd Q{MatrixXd::Zero(A.cols(), A.cols())};
    for (size_t i{0}; i < num_segments; i++) {
        Q.block(i * N, i * N, num_coefficients, num_coefficients) += BuildOmega(spline.time_handler.delta_t_ns_, 1.0);
    }

    MatrixXd const A_n{A.transpose() * A + Q};
    MatrixXd const b_n{A.transpose() * b};
    VectorXd const x{A_n.ldlt().solve(b_n)};  // Solve

    for (int i{0}; i < x.size(); i += 3) {
        spline.control_points.push_back(x.segment<3>(i));
    }

    return spline;
}

std::tuple<MatrixXd, VectorXd> CubicBSplineC3Init::BuildAb(std::vector<C3Measurement> const& measurements,
                                                           size_t const num_segments, TimeHandler const& time_handler) {
    // NOTE(Jack): Is that a formal guarantee we can make somewhere, that all measurements have the same number of
    // states as the control points? Is that implied by splines?
    size_t const measurement_dim{std::size(measurements) * N};
    size_t const num_control_points{num_segments + constants::degree};
    size_t const control_point_dim{num_control_points * N};

    MatrixXd A{MatrixXd::Zero(measurement_dim, control_point_dim)};
    for (size_t j{0}; j < std::size(measurements); ++j) {
        // TODO(Jack): What is a realistic method to deal with the end of a sequence??? Because the last measurement
        // will always have a time stamp at the very end of a time segment, which means it will evaluate to u=1 which is
        // not a valid position.
        // ULTRA HACK! Also see note on unprotected optional access below.
        std::uint64_t time_ns_i{measurements[j].t_ns};
        if (j == std::size(measurements) - 1) {
            time_ns_i -= 1;
        }

        // WARN(Jack): Unprotected optional access, but technically we should always been in a valid time segment
        // because the measurement times are always non-decreasing and set the time limit themselves.
        auto const [u_i, i]{time_handler.SplinePosition(time_ns_i, num_control_points).value()};

        A.block(j * N, i * N, N, num_coefficients) = VectorizeWeights(u_i);
    }

    VectorXd b{VectorXd{measurement_dim, 1}};
    for (size_t i{0}; i < std::size(measurements); ++i) {
        b.segment(i * N, N) = measurements[i].r3;
    }

    return {A, b};
}

CubicBSplineC3Init::ControlPointBlock CubicBSplineC3Init::VectorizeWeights(double const u_i) {
    VectorKd const weights_i{R3Spline::B<DerivativeOrder::Null>(u_i)};

    ControlPointBlock sparse_weights{ControlPointBlock::Zero()};
    for (int i{0}; i < K; ++i) {
        sparse_weights.block(0, i * N, N, N) = Matrix3d::Identity() * weights_i[i];
    }

    return sparse_weights;
}

MatrixXd CubicBSplineC3Init::BuildOmega(std::uint64_t const delta_t_ns, double const lambda) {
    MatrixKd const derivative_op{DerivativeOperator(K) / delta_t_ns};
    MatrixKd const hilbert_matrix{HilbertMatrix(7)};  // Is this the right name? Or is it just coincidentally so?

    // Take the second derivative
    MatrixKd V_i{delta_t_ns * hilbert_matrix};
    for (int i = 0; i < 2; i++) {
        V_i = derivative_op.transpose() * V_i * derivative_op;
    }

    MatrixXd V{MatrixXd::Zero(num_coefficients, num_coefficients)};
    for (int i = 0; i < N; ++i) {
        V.block(i * K, i * K, K, K) = V_i;
    }

    Eigen::MatrixXd const M{VectorizeBlendingMatrix(R3Spline::M_)};
    Eigen::MatrixXd const omega{M.transpose() * V * M};

    return lambda * omega;
}

MatrixXd DerivativeOperator(int const order) {
    MatrixXd D{MatrixXd::Zero(order, order)};
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

MatrixXd VectorizeBlendingMatrix(MatrixKd const& blending_matrix) {
    auto build_block = [](Vector4d const& element) {
        Eigen::Matrix<double, constants::states * constants::order, constants::states> X{
            Eigen::Matrix<double, constants::states * constants::order, constants::states>::Zero()};
        for (int i = 0; i < constants::states; i++) {
            X.block(i * constants::order, i, constants::order, 1) = element;
        }

        return X;
    };

    Eigen::MatrixXd M{
        Eigen::MatrixXd::Zero(constants::order * constants::states, constants::order * constants::states)};
    for (int j{0}; j < constants::order; j++) {
        M.block(0, j * constants::states, constants::states * constants::order, constants::states) =
            build_block(blending_matrix.row(j));
    }

    return M;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::spline
