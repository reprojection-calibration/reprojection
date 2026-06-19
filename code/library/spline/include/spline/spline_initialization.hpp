
#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::spline {

// TODO(Jack): Does this belong in the calibration package? Using Frames here is a little too far away from the core
//  pure spline logic. Or maybe even in the optimization package?
std::pair<Matrix2NXd, TimeHandler> InitializeSe3SplineState(Frames const& frames, int const frequency);

// NOTE(Jack): This was originally intended just for the internal spline interpolation code. But it turns out we also
// need the minimum energy constraint when we are doing the extrinsic optimization itself, otherwise the imu data camera
// frames will get completely out of sync.

// https://www.stat.cmu.edu/~cshalizi/uADA/12/lectures/ch07.pdf
//      "For smoothing splines, using a stiffer material corresponds to increasing lambda"
CoefficientBlock BuildOmega(std::uint64_t const delta_t_ns, double const lambda);

}  // namespace reprojection::spline