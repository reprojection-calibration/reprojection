
#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::spline {

// TODO(Jack): Does this belong in the calibration package? Using Frames here is a little too far away from the core
//  pure spline logic. Or maybe even in the optimization package?
std::pair<Matrix2NXd, TimeHandler> InitializeSe3SplineState(Frames const& frames);

std::pair<MatrixNXd, TimeHandler> InitializeC3SplineState(PositionMeasurements const& measurements,
                                                          size_t const num_segments);

}  // namespace reprojection::spline