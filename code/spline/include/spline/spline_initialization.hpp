#include "spline/constants.hpp"
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
CubicBSplineC3 InitializeSpline(std::vector<C3Measurement> const& measurements, size_t const num_segments);

std::tuple<MatrixXd, VectorXd> BuildAb(std::vector<C3Measurement> const& measurements, size_t const num_segments,
                                       TimeHandler const& time_handler);

// TODO(Jack): Naming here! Technically this is really a "sparse" control point block, does that matter?
// A control point block holds the spline weights in a sparse fashing, that can be multiplied by the control points
// stacked into one vector.
// TODO(Jack): Move this to where we have other type defs?
using ControlPointBlock = Eigen::Matrix<double, constants::states, constants::states * constants::order>;

// TODO(Jack): This name is not really correct, because we are manipulating the control point weights such that they can
// be applied to vectorized control points. We should be more specific that we are actually working on the weights here,
// and not vectorizing them. This is actually more a general tool in helping us "vectorize" the entire problem.
ControlPointBlock VectorizeWeights(double const u_i);

// https://www.stat.cmu.edu/~cshalizi/uADA/12/lectures/ch07.pdf
//      "For smoothing splines, using a stiffer material corresponds to increasing lambda"
// TODO(Jack): Given that the constants are set and fixed, I think we can make a lot of these matrices fixed sizes.
Eigen::MatrixXd BuildOmega(std::uint64_t const delta_t_ns, double const lambda);

// TODO MUST MULTIPLY RETURN BY DELTA T
// For a discussion of the matrix derivative operator of a polynomial space please see the following links:
//      (1) https://math.stackexchange.com/questions/4687306/derivative-as-a-matrix-mathbfd-dfrac-mathrmd-mathrmdx
//      (2) https://math.stackexchange.com/questions/1003358/how-do-you-write-a-differential-operator-as-a-matrix
//
// For order=4 the matrix derivative operator will be a 4x4 matrix with the three elements on the super-diagonal equal
// to [1, 2, 3], which correspond to the first derivative coefficients of the polynomial (a + bx + cx^2 + dx^3)
MatrixXd DerivativeOperator(int const order);

MatrixXd HilbertMatrix(int const size);

// TODO MUST MULTIPLY RETURN BY DELTA T
MatrixXd HankelMatrix(VectorXd const& coefficients);

// See note above in the other "vectorize" function about what is really happening here.
// TODO(Jack): We can definitely use some typedegs of constants to make the matrices easier to read!
// TODO(Jack): Are any of the places where we have constants::states actually supposed to be degree?
MatrixXd VectorizeBlendingMatrix(MatrixKd const& blending_matrix);

}  // namespace reprojection::spline
