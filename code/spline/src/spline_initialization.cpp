#include "spline/spline_initialization.hpp"

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

// We are building a linear problem Ax=b,
//      A: comes from our spline basis matrices
//      x: our unknown evenly spaced control points that created b - we want to find x
//      b: our measurements in the state space (same dimensions state space as the control points)
//
// The smallest problem we can build for a cubic b-spline is; given two measurements and two unique times, interpolate
// the four evenly spaced control points that define a spline which passes through the measurements.
//
// If we have two measurements then we can define one time segment with four control points, if we have three
// measurements we can either still define one time segment as before, or define two time segments now defined by five
// control points. How this works when measurements are missing, or too many or too few time segments are selected is at
// this time not clear.
//
// Our first strategy will be to stack all measurements and control points into vectors. Because we are dealing with a
// simple uniform spline it might happen that later we realize some symmetries that help us reduce the size of the
// problem itself. However, it might also be that because the measurements are not evenly spaced that this is not
// possible.
//      problem dimension: A_6_12 * x_12_1 = b_6_1      - for two measurements defining one time segment

namespace reprojection::spline {

// TODO(Jack): Is it right to use the C3Measurement here? Technically we do not use the derivative information at all,
// and it makse it impossible to use a map because the data is not contigious in memory. WARN(Jack): Expects time sorted
// measurements! Time stamp must be non-decreasing, how can we enforce this?
CubicBSplineC3 InitializeSpline(std::vector<C3Measurement> const& measurements, size_t const num_segments) {
    // TODO(Jack): Will rounding effect the time handling here?
    // TODO(Jack): Given a certain number of measurement is there a limit/boundary to valid num_segments?
    CubicBSplineC3 spline{measurements[0].t_ns, (measurements.back().t_ns - measurements.front().t_ns) / num_segments};
    auto const [A, b]{BuildAb(measurements, num_segments, spline.time_handler)};

    // TODO(Jack): Pass lambda as parameter
    MatrixXd Q{MatrixXd::Zero(A.cols(), A.cols())};
    for (size_t i{0}; i < num_segments; i++) {
        Q.block(constants::states * i, constants::states * i, constants::states * constants::order,
                constants::states * constants::order) += BuildOmega(spline.time_handler.delta_t_ns_, 1.0);
    }

    MatrixXd const A_n{A.transpose() * A + Q};
    MatrixXd const b_n{A.transpose() * b};
    VectorXd const x{A_n.ldlt().solve(b_n)};  // Solve

    for (int i{0}; i < x.size(); i += 3) {
        spline.control_points.push_back(x.segment<3>(i));
    }

    return spline;
}

std::tuple<MatrixXd, VectorXd> BuildAb(std::vector<C3Measurement> const& measurements, size_t const num_segments,
                                       TimeHandler const& time_handler) {
    // NOTE(Jack): Is that a formal guarantee we can make somewhere, that all measurements have the same number of
    // states as the control points? Is that implied by splines?
    size_t const measurement_dim{std::size(measurements) * constants::states};
    size_t const num_control_points{constants::degree + num_segments};
    size_t const control_point_dim{num_control_points * constants::states};

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

        A.block(constants::states * j, constants::states * i, constants::states, constants::states * constants::order) =
            VectorizeWeights(u_i);
    }

    VectorXd b{VectorXd{measurement_dim, 1}};
    for (size_t i{0}; i < std::size(measurements); ++i) {
        b.segment(constants::states * i, constants::states) = measurements[i].r3;
    }

    return {A, b};
}

// TODO(Jack): This name is not really correct, because we are manipulating the control point weights such that they can
// be applied to vectorized control points. We should be more specific that we are actually working on the weights here,
// and not vectorizing them. This is actually more a general tool in helping us "vectorize" the entire problem.
ControlPointBlock VectorizeWeights(double const u_i) {
    VectorKd const weights_i{R3Spline::B<DerivativeOrder::Null>(u_i)};

    ControlPointBlock sparse_weights{ControlPointBlock::Zero()};
    for (int i{0}; i < constants::order; ++i) {
        sparse_weights.block(0, constants::states * i, constants::states, constants::states) =
            Matrix3d::Identity() * weights_i[i];
    }

    return sparse_weights;
}

// https://www.stat.cmu.edu/~cshalizi/uADA/12/lectures/ch07.pdf
//      "For smoothing splines, using a stiffer material corresponds to increasing lambda"
// TODO(Jack): Given that the constants are set and fixed, I think we can make a lot of these matrices fixed sizes.
Eigen::MatrixXd BuildOmega(std::uint64_t const delta_t_ns, double const lambda) {
    MatrixKd const derivative_op{DerivativeOperator(constants::order) / delta_t_ns};
    MatrixXd const hilbert_matrix{HilbertMatrix(7)};  // Is this the right name? Or is it just coincidentally so?

    // Take the second derivative
    MatrixKd V_i{delta_t_ns * hilbert_matrix};
    for (int i = 0; i < 2; i++) {
        V_i = derivative_op.transpose() * V_i * derivative_op;
    }

    MatrixXd V{MatrixXd::Zero(constants::states * constants::order, constants::states * constants::order)};
    for (int i = 0; i < constants::states; ++i) {
        V.block(constants::order * i, constants::order * i, constants::order, constants::order) = V_i;
    }

    Eigen::MatrixXd const M{VectorizeBlendingMatrix(R3Spline::M_)};
    Eigen::MatrixXd const omega{M.transpose() * V * M};

    return lambda * omega;
}

// TODO MUST MULTIPLY RETURN BY DELTA T
// For a discussion of the matrix derivative operator of a polynomial space please see the following links:
//      (1) https://math.stackexchange.com/questions/4687306/derivative-as-a-matrix-mathbfd-dfrac-mathrmd-mathrmdx
//      (2) https://math.stackexchange.com/questions/1003358/how-do-you-write-a-differential-operator-as-a-matrix
//
// For order=4 the matrix derivative operator will be a 4x4 matrix with the three elements on the super-diagonal equal
// to [1, 2, 3], which correspond to the first derivative coefficients of the polynomial (a + bx + cx^2 + dx^3)
MatrixXd DerivativeOperator(int const order) {
    MatrixXd D{MatrixXd::Zero(order, order)};
    D.diagonal(1) = PolynomialCoefficients(order).row(static_cast<int>(DerivativeOrder::First)).rightCols(order - 1);

    return D;
}  // LCOV_EXCL_LINE

MatrixXd HilbertMatrix(int const size) {
    VectorXd const hilbert_coefficients{Eigen::VectorXd::LinSpaced(size, 1, size).cwiseInverse()};

    return HankelMatrix(hilbert_coefficients);
}

// TODO MUST MULTIPLY RETURN BY DELTA T
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

// See note above in the other "vectorize" function about what is really happening here.
// TODO(Jack): We can definitely use some typedegs of constants to make the matrices easier to read!
// TODO(Jack): Are any of the places where we have constants::states actually supposed to be degree?
MatrixXd VectorizeBlendingMatrix(MatrixKd const& blending_matrix) {
    auto build_block = [](Vector4d const& element) {
        Eigen::Matrix<double, constants::states * constants::order, constants::states> X{
            Eigen::Matrix<double, constants::states * constants::order, constants::states>::Zero()};
        for (int i = 0; i < constants::states; i++) {
            X.block(i * constants::order, i, constants::order, 1) = element;
        }
        return X;
    };

    Eigen::MatrixXd M{Eigen::MatrixXd::Zero(constants::order * 3, constants::order * 3)};
    for (int j{0}; j < constants::order; j++) {
        M.block(0, j * constants::states, constants::states * constants::order, constants::states) =
            build_block(blending_matrix.row(j));
    }

    return M;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::spline
