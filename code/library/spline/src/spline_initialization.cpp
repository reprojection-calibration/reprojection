#include "spline/spline_initialization.hpp"

#include "spline/r3_spline.hpp"

#include "cubic_spline_c3_init.hpp"
#include "sparse_utilities.hpp"

namespace reprojection::spline {

// TODO(Jack): Unit test!
// TODO(Jack): Rename frequency to sample_rate_hz? And change type to double?
std::pair<Matrix2NXd, TimeHandler> InitializeSe3SplineState(Frames const& frames, int const frequency) {
    PositionMeasurements so3;
    PositionMeasurements r3;
    for (auto const& [timestamp_ns, frame_i] : frames) {
        so3.insert({timestamp_ns, {frame_i.pose.topRows<N>()}});
        r3.insert({timestamp_ns, {frame_i.pose.bottomRows<N>()}});
    }

    double const delta_t_s{(std::crbegin(frames)->first - std::cbegin(frames)->first) / 1e9};
    int64_t const num_segments{static_cast<int64_t>(frequency * delta_t_s)};

    auto const [so3_control_points, time_handler_a]{InitializeC3SplineState(so3, num_segments)};
    auto const [r3_control_points, time_handler_b]{InitializeC3SplineState(r3, num_segments)};

    if (time_handler_a != time_handler_b) {
        throw std::runtime_error                                                               // LCOV_EXCL_LINE
            ("During se3 spline initialization we somehow got two different time handlers!");  // LCOV_EXCL_LINE
    }

    Matrix2NXd se3_control_points{2 * N, so3_control_points.cols()};
    se3_control_points.topRows<N>() = so3_control_points;
    se3_control_points.bottomRows<N>() = r3_control_points;

    return {se3_control_points, time_handler_a};
}

// NOTE(Jack): Lambda could also be called "stiffness", as it constrains the spline to have minimum energy and fit the
// points stiffly. This is critical for cases where we want to interpolate more poses than we have initial data points.
CoefficientBlock BuildOmega(std::uint64_t const delta_t_ns, double const lambda) {
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

}  // namespace reprojection::spline