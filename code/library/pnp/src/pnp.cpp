#include "pnp/pnp.hpp"

#include "calibration_data_views/optimization_view.hpp"
#include "geometry/lie.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"

#include "dlt.hpp"
#include "plane_utilities.hpp"

namespace reprojection::pnp {

// WARN(Jack): When doing the Dlt22 you are restricted to being in unit image coordinates, therefore we hard code
// the intrinsics and bounds for that case. If however you are doing the Dlt23 case you do not have this distinction
// and are instead required to pass in the bounds and the Dlt23 functions returns a K matrix in the scale of the
// input pixels.
// TODO(Jack): What is the physical meaning of the unit ImageBounds for the Dlt22 case, does this limit us to a 90
//  degree FoV? Because only points that have x/z or y/z ratio greater than one are invalid. Is this really a physically
//  meaningful and correct piece of logic? Or does it just happen to work, what if I chose to set the bounds as -2,+2
//  instead?
PnpResult Pnp(Bundle const& bundle, std::optional<ImageBounds> bounds) {
    Isometry3d tf_co_w;
    Array4d pinhole_intrinsics;
    if (IsPlane(bundle.points) and bundle.pixels.rows() > 4 and not bounds) {
        tf_co_w = Dlt22(bundle);
        pinhole_intrinsics = {1, 1, 0, 0};   // Equivalent to K = I_3x3
        bounds = ImageBounds{-1, 1, -1, 1};  // Unit image dimension bounds
    } else if (bundle.pixels.rows() > 6 and bounds) {
        std::tie(tf_co_w, pinhole_intrinsics) = Dlt23(bundle);
    } else {
        return PnpErrorCode::InvalidDlt;
    }

    // TODO(Jack): This is a heuristic slightly hacky looking way to check if the above DLT algorithm evaluation failed.
    //  If we had a better theoretical algorithmic understanding of what causes these failures and how we can detect
    //  them then we could improve this code here.
    Array6d const aa_co_w{geometry::Log(tf_co_w)};
    if (aa_co_w.hasNaN()) {
        return PnpErrorCode::ContainsNan;
    }

    // TODO(Jack): The optimizer should be configured to keep the intrinsics constant here!
    // TODO(Jack): Building an entire CameraCalibrationData here just to optimize the one camera frame seems a little
    //  like overkill, but using the common interface here and in the primary optimization is just cleaner.
    // NOTE(Jack): We hardcode the one frame here to have a timestamps of 0 so we can access it below.
    CameraCalibrationData data{
        {"", CameraModel::Pinhole, bounds.value()}, pinhole_intrinsics, {}, {{0, {{bundle, {}}, aa_co_w}}}};
    optimization::CameraNonlinearRefinement(OptimizationDataView(data));

    auto const opt_tf_co_w{data.frames.at(0).optimized_pose};
    if (opt_tf_co_w) {
        return geometry::Exp(opt_tf_co_w.value());
    } else {
        // NOTE(Jack): I do not know if it is really possible to ever reach this condition because theoretically we
        // should only get to CameraNonlinearRefinement if we have a good initial pose. Because we initialize the
        // optimized pose with the initial pose in CameraNonlinearRefinement it almost guarantees that this condition
        // will never be triggered. For now at least... We leave this conditional here because we need to check the
        // optional anyway!
        return PnpErrorCode::FailedRefinement;  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::pnp