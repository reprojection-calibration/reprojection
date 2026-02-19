#include "cubic_spline_c3_init.hpp"

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

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
        // ERROR(Jack): At this time we have no principled strategy to deal with the end conditions of the spline,
        // therefore we need this hack here. What this hack does is ensure that at the very end of the spline, when
        // time_ns_i is at the end (corresponds to the last measurement), it does not start another time segment past
        // the end of the spline, but instead stays on the last valid segment at the very end (ex. u_i=0.99999). This is
        // definitely a hack, but it "works"!
        std::uint64_t time_ns_i{measurements[j].t_ns};
        if (j == std::size(measurements) - 1) {
            time_ns_i -= static_cast<std::uint64_t>(1 + 0.01 * time_handler.delta_t_ns_);
        }

        // WARN(Jack): Unprotected optional access! Technically we should always been in a valid time segment because
        // the measurement times are always non-decreasing and define the limit of the valid times themselves. In
        // combination with the hack described above, we should not get problems here. However, in reality this shows
        // that maybe we are not describing or capturing the problem well. A better solution here is welcome!
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

// NOTE(Jack): Lambda could also be called "stiffness", as it constrains the spline to have minimum energy and fit the
// points stiffly. This is critical for cases where we want to interpolate more poses than we have initial data points.
CubicBSplineC3Init::CoefficientBlock CubicBSplineC3Init::BuildOmega(std::uint64_t const delta_t_ns,
                                                                    double const lambda) {
    MatrixKd const derivative_op{DerivativeOperator(K) / delta_t_ns};
    // NOTE(Jack): This is a hilbert matrix, but is it just coincidentally so? Or is there a better name that better
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

    static CoefficientBlock const M{BlockifyBlendingMatrix(R3Spline::M_)};
    CoefficientBlock const omega{M.transpose() * V * M};

    return lambda * omega;
}

CubicBSplineC3Init::CoefficientBlock CubicBSplineC3Init::BlockifyBlendingMatrix(MatrixKd const& blending_matrix) {
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
