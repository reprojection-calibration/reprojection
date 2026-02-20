#include "spline/constants.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

// We are building a linear problem Ax=b,
//      A: constructed from the spline basis matrices
//      x: our unknown evenly spaced control points that created b - our goal is to solve for this
//      b: our measurements in the state space (same dimensions state space as the control points)
//
// The smallest problem we can build for a cubic b-spline is; given two measurements and two unique times, interpolate
// the four evenly spaced control points that define a spline with one time interval which passes through the
// measurements.
//
// If we have two measurements then we can define one time segment with four control points, if we have three
// measurements we can either still define one time segment as before, or define two time segments now defined by five
// control points.
//
// Our first strategy will be to stack all measurements and control points into vectors. Because we are dealing with a
// simple uniform spline it might happen that later we realize some symmetries that help us reduce the size of the
// problem itself. However, it might also be that because the measurements are not evenly spaced that this is not
// possible.
//      problem dimension: A_6_12 * x_12_1 = b_6_1      - for two measurements defining one time segment

namespace reprojection::spline {

struct CubicBSplineC3Init {
    static std::tuple<MatrixXd, VectorXd> BuildAb(PositionMeasurements const& positions, size_t const num_segments,
                                                  TimeHandler const& time_handler);

    /**
     * \brief How many control points are required to evaluate the spline (=3 for cubic spline).
     */
    static int constexpr K{constants::order};

    /**
     * \brief The size of the state space (=3 for both R3 and so3, translation and rotation).
     */
    static int constexpr N{constants::states};

    /**
     * \brief Length of a vectorized control point block (=12 for a cubic b-spline with 3D state space).
     *
     * NOTE(Jack): We are entering a mixed terminology space because in the context of splines the coefficients often
     * refer to the values multiplied by the control points. Here however we are actually referring to the control
     * points themselves as coefficients. And further in the code we refer to what we normally would call the spline
     * coefficients as "weights". This is a confusing aspect that should be addressed if it causes problems.
     */
    static int constexpr num_coefficients{K * N};

    /**
     * \brief A matrix used to hold the sparsified/diagonalized spline weights.
     *
     * A matrix of shape (N x num_coefficients) holding the sparsified/diagonalized spline weights can be directly
     * multiplied by a vectorized control points block (a vector with length=num_coefficients) to evaluate the spline.
     */
    using ControlPointBlock = Eigen::Matrix<double, N, num_coefficients>;

    using CoefficientBlock = Eigen::Matrix<double, num_coefficients, num_coefficients>;

    // TODO(Jack): Is weights really the right term here? We are blockifying the entire B vector which combines both the
    //  basis matrix contribution and the time weighting.
    static ControlPointBlock BlockifyWeights(double const u_i);

    // https://www.stat.cmu.edu/~cshalizi/uADA/12/lectures/ch07.pdf
    //      "For smoothing splines, using a stiffer material corresponds to increasing lambda"
    static CoefficientBlock BuildOmega(std::uint64_t const delta_t_ns, double const lambda);

    static CoefficientBlock BlockifyBlendingMatrix(MatrixKd const& blending_matrix);
};

// For a discussion of the matrix derivative operator of a polynomial space please see the following links:
//      (1) https://math.stackexchange.com/questions/4687306/derivative-as-a-matrix-mathbfd-dfrac-mathrmd-mathrmdx
//      (2) https://math.stackexchange.com/questions/1003358/how-do-you-write-a-differential-operator-as-a-matrix
//
// For order=4 the matrix derivative operator will be a 4x4 matrix with the three elements on the super-diagonal equal
// to [1, 2, 3], which correspond to the first derivative coefficients of the polynomial (a + bx + cx^2 + dx^3)
MatrixXd DerivativeOperator(int const order);

MatrixXd HilbertMatrix(int const size);

MatrixXd HankelMatrix(VectorXd const& coefficients);

}  // namespace reprojection::spline
