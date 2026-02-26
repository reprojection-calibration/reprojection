#include "spline/spline_initialization.hpp"

#include <Eigen/SparseCholesky>

#include "cubic_spline_c3_init.hpp"
#include "sparse_utilities.hpp"

namespace reprojection::spline {

// TODO(Jack): Is 20 times the number of frames high enough num_segments for the imu data frequency? Do we need to
//  parameterize this?
std::pair<Matrix2NXd, TimeHandler> InitializeSe3SplineState(Frames const& frames) {
    PositionMeasurements so3;
    PositionMeasurements r3;
    for (auto const& [timestamp_ns, frame_i] : frames) {
        so3.insert({timestamp_ns, {frame_i.pose.topRows<constants::states>()}});
        r3.insert({timestamp_ns, {frame_i.pose.bottomRows<constants::states>()}});
    }

    size_t const num_segments{20 * std::size(frames)};
    auto const [so3_control_points, time_handler_a]{InitializeC3SplineState(so3, num_segments)};
    auto const [r3_control_points, time_handler_b]{InitializeC3SplineState(r3, num_segments)};

    if (time_handler_a != time_handler_b) {
        throw std::runtime_error(
            "During se3 spline initialization we somehow got two different time handlers!");  // LCOV_EXCL_LINE
    }

    Matrix2NXd se3_control_points{2 * constants::states, so3_control_points.cols()};
    se3_control_points.topRows<constants::states>() = so3_control_points;
    se3_control_points.bottomRows<constants::states>() = r3_control_points;

    return {se3_control_points, time_handler_a};
}

using CoefficientBlock = CubicBSplineC3Init::CoefficientBlock;

std::pair<MatrixNXd, TimeHandler> InitializeC3SplineState(PositionMeasurements const& measurements,
                                                          size_t const num_segments) {
    // WARN(Jack): We might have some rounding error here due calculating delta_t_ns, at this time that is no known
    // problem.
    uint64_t const t0_ns{std::cbegin(measurements)->first};
    uint64_t const tn_ns{std::crbegin(measurements)->first};
    uint64_t const delta_t_ns{(tn_ns - t0_ns) / num_segments};
    TimeHandler const time_handler{t0_ns, delta_t_ns};

    auto const [A, b]{CubicBSplineC3Init::BuildAb(measurements, num_segments, time_handler)};

    // NOTE(Jack): At this time lambda here is hardcoded, it might make sense at some time in the future to parameterize
    // this, but currently I see no scenario where we can really expect the user to parameterize it, so we leave it
    // hardcoded for now.
    // NOTE(Jack): The lambda that you need to use is very large, about e7/e8/e9 magnitude because we use nanoseconds
    // timestamps which results in very small values in the omega matrix otherwise.
    CoefficientBlock const omega{CubicBSplineC3Init::BuildOmega(delta_t_ns, 1e8)};
    Eigen::SparseMatrix<double> const Q{DiagonalSparseMatrix(omega, CubicBSplineC3Init::N, num_segments)};

    // NOTE(Jack): When we first tried to apply this to larger spline initialization problems (ex. 2000 segments) it was
    // slow as hell and took about 55 seconds on my laptop to initialize the rotation and translation. But then I used a
    // sparse solver and it cut the time down to about 600ms. Therefore I think we are on the right track here using
    // sparse logic.
    // WARN(Jack): In the documentation for the .sparseView() method it says "This method is typically used when
    // prototyping to convert a quickly assembled dense Matrix D to a SparseMatrix S". That sentence implies it should
    // only be used for prototyping, but it solved my problem (initialization time cut from 55s to 600ms) so honestly I
    // am asking myself why I should refactor the entire initialization code to be "sparse by default" and not use dense
    // matrices like we do during the problem construction. Maybe it would be nice to transfer all the code directly to
    // work on the sparse representation, but at this time I see not benefit.
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

    // TODO(Jack): Is there a better way to calculate the number of control points here than x.rows()/3?
    return {Eigen::Map<MatrixNXd const>(x.data(), constants::states, x.rows() / constants::states), time_handler};
}

}  // namespace reprojection::spline