#include "pnp/pnp.hpp"

#include "calibration_data_views/optimization_view.hpp"
#include "dlt.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "plane_utilities.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::pnp {

// WARN(Jack): When doing the Dlt22 you are restricted to being in unit image coordinates, therefore we hard code
// the intrinsics and bounds for that case. If however you are doing the Dlt23 case you do not have this distinction
// and are instead required to pass in the bounds and the Dlt23 functions returns a K matrix in the scale of the
// input pixels.
PnpResult Pnp(Bundle const& bundle, std::optional<ImageBounds> bounds) {
    Isometry3d tf;
    Array4d pinhole_intrinsics;
    if (IsPlane(bundle.points) and bundle.pixels.rows() > 4) {
        tf = Dlt22(bundle);
        pinhole_intrinsics = {1, 1, 0, 0};   // Equivalent to K = I_3x3
        bounds = ImageBounds{-1, 1, -1, 1};  // Unit image dimension boundss
    } else if (bundle.pixels.rows() > 6 and bounds) {
        std::tie(tf, pinhole_intrinsics) = Dlt23(bundle);
    } else {
        return PnpErrorCode::InvalidDlt;
    }

    // TODO(Jack): This is a heuristic slightly hacky looking way to check if the above DLT algorithm evaluation failed.
    //  If we had a better theoretical algorithmic understanding of what causes these failures and how we can detect
    //  them then we could improve this code here.
    Array6d const se3{geometry::Log(tf)};
    if (se3.hasNaN()) {
        return PnpErrorCode::ContainsNan;
    }

    // TODO(Jack): The optimizer should be configured to keep the intrinsics constant here!
    CameraCalibrationData data{
        {"", CameraModel::Pinhole, bounds.value()}, pinhole_intrinsics, {}, {{0, {{bundle, {}}, geometry::Log(tf)}}}};
    optimization::CameraNonlinearRefinement(OptimizationDataView(data));

    // NOTE(Jack): There is only one single frame here in the CameraCalibrationData structure, therefore we can simply
    // use std::cbegin to access the result. In the general case this is not meaningful or correct.
    auto const optimized_pose{std::cbegin(data.frames)->second.optimized_pose};
    if (optimized_pose) {
        return geometry::Exp(optimized_pose.value());
    } else {
        return PnpErrorCode::FailedRefinement;
    }
}

}  // namespace reprojection::pnp