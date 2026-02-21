
#include "spline/spline_state.hpp"
#include "types/spline_types.hpp"

namespace reprojection::spline {

CubicBSplineC3 InitializeC3Spline(PositionMeasurements const& measurements, size_t const num_segments);

}  // namespace reprojection::spline