#include "spline/spline_initialization.hpp"

#include <gtest/gtest.h>

#include "spline/r3_spline.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::spline;

TEST(SplineSplineInitialization, TestInitializeSpline) {
    std::vector<C3Measurement> const measurements{{5000, {0, 0, 0}, DerivativeOrder::Null},  //
                                                  {5100, {1, 1, 1}, DerivativeOrder::Null},
                                                  {5200, {2, 2, 2}, DerivativeOrder::Null}};

    CubicBSplineC3 const one_segment_spline{CubicBSplineC3Init::InitializeSpline(measurements, 1)};
    EXPECT_EQ(one_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(one_segment_spline.time_handler.delta_t_ns_, 200);
    EXPECT_EQ(std::size(one_segment_spline.control_points), 4);
    // NOTE(Jack): At this point this and below are canary in the coal mine tests, to make sure nothing changes as we
    // refactor. An unsolved problem is the time handling, and this is the reason why these values are not exact values
    // on the integers, which given the test data they should be.
    EXPECT_TRUE(one_segment_spline.control_points[0].isApprox(Vector3d{-1.97611, -1.97611, -1.97611}, 1e-6));
    EXPECT_TRUE(one_segment_spline.control_points[3].isApprox(Vector3d{4.05394, 4.05394, 4.05394}, 1e-6));

    CubicBSplineC3 const two_segment_spline{CubicBSplineC3Init::InitializeSpline(measurements, 2)};
    EXPECT_EQ(two_segment_spline.time_handler.t0_ns_, 5000);
    EXPECT_EQ(two_segment_spline.time_handler.delta_t_ns_, 100);
    EXPECT_EQ(std::size(two_segment_spline.control_points), 5);
    // See note above on canary coal mine.
    EXPECT_TRUE(two_segment_spline.control_points[0].isApprox(Vector3d{-0.997462, -0.997462, -0.997462}, 1e-6));
    EXPECT_TRUE(two_segment_spline.control_points[4].isApprox(Vector3d{3.02269, 3.02269, 3.02269}, 1e-6));
}

TEST(SplineSplineInitialization, TestBuildAb) {
    std::vector<C3Measurement> const measurements{{5000, {0, 0, 0}, DerivativeOrder::Null},  //
                                                  {5100, {1, 1, 1}, DerivativeOrder::Null},
                                                  {5200, {2, 2, 2}, DerivativeOrder::Null}};

    int const num_segments{2};
    TimeHandler const time_handler{5000, 100, constants::order};

    auto const [A, b]{CubicBSplineC3Init::BuildAb(measurements, num_segments, time_handler)};
    EXPECT_EQ(A.rows(), 9);
    EXPECT_EQ(A.cols(), 15);
    EXPECT_EQ(b.rows(), 9);

    // TODO(Jack): At this point the actual time handling logic inside the function is not at all/or well tested.
    // Can we test that from this view? Or is that already tested somehwere else?
}

TEST(SplineSplineInitialization, TestVectorizeWeights) {
    // At u=0 only the first three blocks will have weight - print out the weights to understand the values!
    CubicBSplineC3Init::ControlPointBlock const w_0{CubicBSplineC3Init::VectorizeWeights(0.0)};
    EXPECT_FLOAT_EQ(w_0.block(0, 3, 3, 3).sum(), 3 * (2.0 / 3));  // Second block has all the 2/3 weight elements
    EXPECT_TRUE(w_0.block(0, 9, 3, 3).isZero());                  // Last block is empty

    // At u=1 (in this case 0.9999 because we only have on time segment) the last three block will have values
    CubicBSplineC3Init::ControlPointBlock const w_1{CubicBSplineC3Init::VectorizeWeights(0.99999999999)};
    EXPECT_TRUE(w_1.block(0, 0, 3, 3).isZero());                  // First block is empty
    EXPECT_FLOAT_EQ(w_1.block(0, 6, 3, 3).sum(), 3 * (2.0 / 3));  // Third block has all the 2/3 weights
}

TEST(SplineSplineInitialization, TestBuildOmega) {
    MatrixXd const Q_1{BuildOmega(1, 1)};
    EXPECT_EQ(Q_1.rows(), 12);
    EXPECT_EQ(Q_1.cols(), 12);
    EXPECT_FLOAT_EQ(Q_1.diagonal().sum(), 8);  // Heuristic!

    MatrixXd const Q_100{BuildOmega(100, 1)};
    EXPECT_FLOAT_EQ(Q_100.diagonal().sum(), 8e-6);
}

TEST(SplineSplineInitialization, TestDerivativeOperator) {
    Matrix3d const D3{DerivativeOperator(3)};
    EXPECT_TRUE(D3.diagonal(1).isApprox(Vector2d{1, 2}));

    // This should be the size used for the cubic b-spline
    MatrixKd const D4{DerivativeOperator(constants::order)};
    EXPECT_TRUE(D4.diagonal(1).isApprox(Vector3d{1, 2, 3}));
}

TEST(SplineSplineInitialization, TestHankelMatrix) {
    Vector3d const coefficients{1, 2, 3};
    MatrixXd const hankel_matrix{HankelMatrix(coefficients)};
    EXPECT_EQ(hankel_matrix.rows(), 2);
    EXPECT_EQ(hankel_matrix.cols(), 2);
    EXPECT_EQ(hankel_matrix(0, 1), 2);

    // The type of hankel matrix actually used by the b-spline initialization - a hilbert matrix
    MatrixXd const hilbert_matrix{HilbertMatrix(7)};
    EXPECT_EQ(hilbert_matrix.rows(), 4);
    EXPECT_EQ(hilbert_matrix.cols(), 4);
    EXPECT_EQ(hilbert_matrix(0, 3), 0.25);
}

TEST(SplineSplineInitialization, TestVectorizeBlendingMatrix) {
    MatrixKd const blending_matrix{R3Spline::M_};
    MatrixXd const vectorized{VectorizeBlendingMatrix(blending_matrix)};  // TODO(Jack): Better name than vectorized!

    EXPECT_EQ(vectorized.rows(), 12);
    EXPECT_EQ(vectorized.cols(), 12);
    EXPECT_TRUE(vectorized.topLeftCorner(4, 1).isApprox(blending_matrix.row(0).transpose()));
    EXPECT_TRUE(vectorized.bottomRightCorner(4, 1).isApprox(blending_matrix.row(3).transpose()));
}
