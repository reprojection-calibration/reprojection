#include "cubic_spline_c3_init.hpp"

#include <gtest/gtest.h>

#include "spline/r3_spline.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::spline;

TEST(SplineSplineInitialization, TestBuildAb) {
    PositionMeasurements const measurements{{5000, {{0, 0, 0}}},  //
                                            {5100, {{1, 1, 1}}},
                                            {5200, {{2, 2, 2}}}};

    int const num_segments{2};
    TimeHandler const time_handler{5000, 100, constants::order};

    auto const [A, b]{CubicBSplineC3Init::BuildAb(measurements, num_segments, time_handler)};
    EXPECT_EQ(A.rows(), 9);
    EXPECT_EQ(A.cols(), 15);
    EXPECT_EQ(b.rows(), 9);

    // TODO(Jack): At this point the actual time handling logic inside the function is not at all/or well tested.
    // Can we test that from this here? See the notes in the implementation to better understand the open problems there
    // regarding valid data and end conditions.
}

TEST(SplineSplineInitialization, TestVectorizeWeights) {
    // At u=0 only the first three blocks will have weight - print out the weights to understand the values!
    CubicBSplineC3Init::ControlPointBlock const w_0{CubicBSplineC3Init::BlockifyWeights(0.0)};
    EXPECT_FLOAT_EQ(w_0.block(0, 3, 3, 3).sum(), 3 * (2.0 / 3));  // Second block has all the 2/3 weight elements
    EXPECT_TRUE(w_0.block(0, 9, 3, 3).isZero());                  // Last block is empty

    // At u=1 (in this case 0.9999 because we only have on time segment) the last three block will have values
    CubicBSplineC3Init::ControlPointBlock const w_1{CubicBSplineC3Init::BlockifyWeights(0.99999999999)};
    EXPECT_TRUE(w_1.block(0, 0, 3, 3).isZero());                  // First block is empty
    EXPECT_FLOAT_EQ(w_1.block(0, 6, 3, 3).sum(), 3 * (2.0 / 3));  // Third block has all the 2/3 weights
}

TEST(SplineSplineInitialization, TestBuildOmega) {
    MatrixXd const Q_1{CubicBSplineC3Init::BuildOmega(1, 1)};
    EXPECT_EQ(Q_1.rows(), 12);
    EXPECT_EQ(Q_1.cols(), 12);
    EXPECT_FLOAT_EQ(Q_1.diagonal().sum(), 8);  // Heuristic!

    MatrixXd const Q_100{CubicBSplineC3Init::BuildOmega(100, 1)};
    EXPECT_FLOAT_EQ(Q_100.diagonal().sum(), 8e-6);
}

TEST(SplineSplineInitialization, TestVectorizeBlendingMatrix) {
    MatrixKd const blending_matrix{R3Spline::M_};
    MatrixXd const blockified{CubicBSplineC3Init::BlockifyBlendingMatrix(blending_matrix)};

    EXPECT_EQ(blockified.rows(), 12);
    EXPECT_EQ(blockified.cols(), 12);
    EXPECT_TRUE(blockified.topLeftCorner(4, 1).isApprox(blending_matrix.row(0).transpose()));
    EXPECT_TRUE(blockified.bottomRightCorner(4, 1).isApprox(blending_matrix.row(3).transpose()));
}

TEST(SplineSplineInitialization, TestDerivativeOperator) {
    Matrix3d const D3{DerivativeOperator(3)};
    EXPECT_TRUE(D3.diagonal(1).isApprox(Vector2d{1, 2}));

    // This is the version used by the cubic b-spline
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
