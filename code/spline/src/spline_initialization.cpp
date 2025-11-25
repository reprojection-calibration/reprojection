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

    // TODO(Jack): At this time lambda here is hardcoded, it might make sense at some time in the future to paramaterize
    // this, but currently I see no scenario where we can really expect the user to parameterize it, so we leave it
    // hardcoded for now.
    CoefficientBlock const omega{BuildOmega(spline.time_handler.delta_t_ns_, 1e7)};

    MatrixXd Q{MatrixXd::Zero(A.cols(), A.cols())};
    for (size_t i{0}; i < num_segments; i++) {
        Q.block(i * N, i * N, num_coefficients, num_coefficients) += omega;
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
    // NOTE(Jack): For both measurement_dim and control_point_dim we are talking about the "vectorized" dimensions.
    // This means how many values are there when we stack all the individual vectors (i.e. measurements or
    // control points) into one big vector to be used in the Ax=b problem. There x is the control points vector of
    // length control_point_dim and b is the measurement vector of length measurement_dim.
    size_t const measurement_dim{std::size(measurements) * N};
    size_t const num_control_points{num_segments + constants::degree};
    size_t const control_point_dim{num_control_points * N};

    MatrixXd A{MatrixXd::Zero(measurement_dim, control_point_dim)};
    for (size_t j{0}; j < std::size(measurements); ++j) {
        // ERROR(Jack): What is a realistic method to deal with the end of a sequence??? Because the last measurement
        // will always have a time stamp at the very end of a time segment, which means it will evaluate to u=1 which is
        // not a valid position.
        // ULTRA HACK! Also see note on unprotected optional access below.
        std::uint64_t time_ns_i{measurements[j].t_ns};
        if (j == std::size(measurements) - 1) {
            // TODO(Jack): How much do we need to clamp here? Should this clamping be a percentage of the delta_t?
            time_ns_i -= static_cast<std::uint64_t>(1 + 0.01 * time_handler.delta_t_ns_);
        }

        // WARN(Jack): Unprotected optional access, but technically we should always been in a valid time segment
        // because the measurement times are always non-decreasing and set the time limit themselves.
        auto const [u_i, i]{time_handler.SplinePosition(time_ns_i, num_control_points).value()};

        A.block(j * N, i * N, N, num_coefficients) = BlockifyWeights(u_i);
    }

    VectorXd b{VectorXd{measurement_dim, 1}};
    for (size_t i{0}; i < std::size(measurements); ++i) {
        b.segment(i * N, N) = measurements[i].r3;
    }

    return {A, b};
}

CubicBSplineC3Init::ControlPointBlock CubicBSplineC3Init::BlockifyWeights(double const u_i) {
    VectorKd const weights_i{R3Spline::B<DerivativeOrder::Null>(u_i)};

    ControlPointBlock sparse_weights{ControlPointBlock::Zero()};
    for (int i{0}; i < K; ++i) {
        sparse_weights.block(0, i * N, N, N) = Matrix3d::Identity() * weights_i[i];
    }

    return sparse_weights;
}

CubicBSplineC3Init::CoefficientBlock CubicBSplineC3Init::BuildOmega(std::uint64_t const delta_t_ns,
                                                                    double const lambda) {
    MatrixKd const derivative_op{DerivativeOperator(K) / delta_t_ns};
    // NOTE(Jack): This is a hilbert matrix, but is it just conicdentally so? Or is there a better name that better
    // reflects its role in taking the matrix second derivative below?
    static MatrixKd const hilbert_matrix{HilbertMatrix(7)};

    // Take the second derivative
    MatrixKd V_i{delta_t_ns * hilbert_matrix};
    for (int i = 0; i < 2; i++) {
        V_i = derivative_op.transpose() * V_i * derivative_op;
    }

    CoefficientBlock V{CoefficientBlock::Zero()};
    for (int i = 0; i < N; ++i) {
        V.block(i * K, i * K, K, K) = V_i;
    }

    static CoefficientBlock const M{VectorizeBlendingMatrix(R3Spline::M_)};
    CoefficientBlock const omega{M.transpose() * V * M};

    return lambda * omega;
}

CubicBSplineC3Init::CoefficientBlock CubicBSplineC3Init::VectorizeBlendingMatrix(MatrixKd const& blending_matrix) {
    auto build_block = [](Vector4d const& element) {
        Eigen::Matrix<double, num_coefficients, N> X{Eigen::Matrix<double, num_coefficients, N>::Zero()};
        for (int i = 0; i < N; i++) {
            X.block(i * K, i, K, 1) = element;
        }

        return X;
    };

    CoefficientBlock M{CoefficientBlock::Zero()};
    for (int i{0}; i < K; ++i) {
        M.block(0, i * N, num_coefficients, N) = build_block(blending_matrix.row(i));
    }

    return M;
}  // LCOV_EXCL_LINE

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

}  // namespace reprojection::spline
