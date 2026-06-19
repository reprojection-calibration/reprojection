#pragma once

namespace reprojection::spline::constants {

// NOTE(Jack): Instead of templating everything like mad, we will use this global to parameterize the order of the
// spline. We might regret this later because we might want to switch to a higher order spline just by changing the
// value of the constant here, but then discover that our "generic" code is actually hardcoded for order=4 :(

// NOTE(Jack): See the online https://nurbscalculator.in/ to examine how changing the degree effects the required number
// of control points and knots.
// NOTE(Jack): In "A Tutorial on Uniform B-Spline" by Yi Zhou (https://arxiv.org/pdf/2309.15477) section "4 FAQs" calls
// out "Efficient derivative computation for cumulative b-splines on lie groups" for treating knows and control points
// identically, which is not the case. As Yi Zhou points out, control points are design space constraints (ex. an SE3
// pose) and knots are a list of positions in the parametric time domain t_i range [0,1]. For uniform B-splines these
// knot positions are known and can be calculated as fixed values once the order is known.
inline constexpr int order{4};           // (K) Number of control points required to evaluate the spline
inline constexpr int degree{order - 1};  // (D) Polynomial degree (ex. order=4 --> degree=3, i.e. "cubic spline")
inline constexpr int states{3};          // (N) Size of the state space for both R3 and so3 splines

// NOTE(Jack): This is primarily used in the spline initialization logic. My initial intent was to keep higher level
// terms like this confined to the private interface of the spline module. But then I needed the spline smoothness
// function "omega" in the optimization which uses this constant value. Therefore, we made one central definition and
// use it wherever needed. If the spline initialization logic one day gets made private again we can hide this too.
/**
 * \brief Length of a vectorized control point block (=12 for a cubic b-spline with 3D state space).
 *
 * NOTE(Jack): We are entering a mixed terminology space because in the context of splines the coefficients often
 * refer to the values multiplied by the control points. Here however we are actually referring to the control
 * points themselves as coefficients. And further in the code we refer to what we normally would call the spline
 * coefficients as "weights". This is a confusing aspect that should be addressed if it causes problems.
 */
inline constexpr int num_coefficients{order * states};  // Number of control points coefficients for one spline segment

}  // namespace reprojection::spline::constants