#pragma once

namespace reprojection::spline::constants {

// Instead of templating everything like mad we will use this global to parameterize the order of the spline.
inline constexpr int k{4};  // Spline order - note spline "degree" is k-1, so when k=4 it is a cubic spline!

}  // namespace reprojection::spline::constants