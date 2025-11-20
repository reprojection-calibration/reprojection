#include <gtest/gtest.h>

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

// TODO(Jack): Naming here! Technically this is really a "sparse" control point block, does that matter?
// A control point block holds the spline weights in a sparse fashing, that can be multiplied by the control points
// stacked into one vector.
using ControlPointBlock = Eigen::Matrix<double, constants::states, constants::states * constants::order>;

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

// TODO(Jack): Is it right to use the C3Measurement here? Technically we do not use the derivative information at all,
// and it makse it impossible to use a map because the data is not contigious in memory. WARN(Jack): Expects time sorted
// measurements! Time stamp must be non-decreasing, how can we enforce this?
CubicBSplineC3 InitializeSpline(std::vector<C3Measurement> const& measurements, size_t const num_segments) {
    // TODO(Jack): Will rounding effect the time handling here?
    // TODO(Jack): Given a certain number of measurement is there a limit/boundary to valid num_segments?
    CubicBSplineC3 spline{measurements[0].t_ns, (measurements.back().t_ns - measurements.front().t_ns) / num_segments};

    auto const [A, b]{BuildAb(measurements, num_segments, spline.time_handler)};

    std::cout << A << std::endl;
    std::cout << b << std::endl;

    return spline;
}

}  // namespace reprojection::spline

using namespace reprojection::spline;

TEST(SplineSplineInitialization, TestBuildAb) {
    std::vector<C3Measurement> const measurements{{5000, {0, 0, 0}, DerivativeOrder::Null},  //
                                                  {5100, {1, 1, 1}, DerivativeOrder::Null},
                                                  {5200, {2, 2, 2}, DerivativeOrder::Null}};

    int const num_segments{2};
    TimeHandler const time_handler{5000, 100, constants::order};

    auto const [A, b]{BuildAb(measurements, num_segments, time_handler)};
    EXPECT_EQ(A.rows(), 9);
    EXPECT_EQ(A.cols(), 15);
    EXPECT_EQ(b.rows(), 9);

    // TODO(Jack): At this point the actual time handling logic inside the function is not at all/or well tested. Can we
    // test that from this view? Or is that already tested somehwere else?
}

// TODO(Jack): reorder tests and methods later during file split
TEST(SplineSplineInitialization, TestTimeHandling) {
    std::vector<C3Measurement> const measurements{{5000, {0, 0, 0}, DerivativeOrder::Null},  //
                                                  {5100, {1, 1, 1}, DerivativeOrder::Null},
                                                  {5200, {2, 2, 2}, DerivativeOrder::Null}};

    CubicBSplineC3 const one_segment_spline{InitializeSpline(measurements, 1)};
    EXPECT_EQ(one_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(one_segment_spline.time_handler.delta_t_ns_, 200);

    CubicBSplineC3 const two_segment_spline{InitializeSpline(measurements, 2)};
    EXPECT_EQ(two_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(two_segment_spline.time_handler.delta_t_ns_, 100);
}

TEST(SplineSplineInitialization, TestVectorizeWeights) {
    // At u=0 only the first three blocks will have weight - print out the weights to understand the values!
    ControlPointBlock const w_0{VectorizeWeights(0.0)};
    ASSERT_FLOAT_EQ(w_0.block(0, 3, 3, 3).sum(), 3 * (2.0 / 3));  // Second block has all the 2/3 weight elements
    ASSERT_TRUE(w_0.block(0, 9, 3, 3).isZero());                  // Last block is empty

    // At u=1 (in this case 0.9999 because we only have on time segment) the last three block will have values
    ControlPointBlock const w_1{VectorizeWeights(0.99999999999)};
    ASSERT_TRUE(w_1.block(0, 0, 3, 3).isZero());                  // First block is empty
    ASSERT_FLOAT_EQ(w_1.block(0, 6, 3, 3).sum(), 3 * (2.0 / 3));  // Third block has all the 2/3 weights
}