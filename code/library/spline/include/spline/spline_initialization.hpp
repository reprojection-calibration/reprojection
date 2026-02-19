
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// TODO(Jack): Is it right to use the C3Measurement here? Technically we do not use the derivative information at
//  all, and it makes it impossible to use a map because the data is not contiguous in memory.
// WARN(Jack): Expects time sorted measurements! Time stamp must be non-decreasing, how can we enforce this?
CubicBSplineC3 InitializeC3Spline(std::vector<C3Measurement> const& measurements, size_t const num_segments);

}  // namespace reprojection::spline