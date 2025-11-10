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
inline constexpr int order{4};           // Number of control points required to evaluate the spline
inline constexpr int degree{order - 1};  // Polynomial degree (ex. order=4 --> degree=3, i.e. "cubic spline")

}  // namespace reprojection::spline::constants