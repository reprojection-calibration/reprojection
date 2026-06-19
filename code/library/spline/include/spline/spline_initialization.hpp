
#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::spline {

// TODO(Jack): Does this belong in the calibration package? Using Frames here is a little too far away from the core
//  pure spline logic. Or maybe even in the optimization package?
std::pair<Matrix2NXd, TimeHandler> InitializeSe3SplineState(Frames const& frames, int const frequency);

// https://www.stat.cmu.edu/~cshalizi/uADA/12/lectures/ch07.pdf
//      "For smoothing splines, using a stiffer material corresponds to increasing lambda"
CoefficientBlock BuildOmega(std::uint64_t const delta_t_ns, double const lambda);

}  // namespace reprojection::spline