#include "pnp/pnp.hpp"

#include "geometry/lie.hpp"
#include "optimization/bundle_adjustment.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"

#include "dlt.hpp"
#include "plane_utilities.hpp"

namespace reprojection::pnp {

// SPLIT AND MOVE FILES!!!
optimization::BaProblem BuildPnpBaProblem(CameraInfo const& camera_info, Bundle const& bundle,
                                          Array3d const& pinhole_intrinsics, Array6d const& se3_co_w) {
    // For the pnp problem we set the extrinsic to identity (i.e. rig==co) and do not optimize the extrinsic or
    // intrinsic. We are only optimizing the world pose.
    optimization::BaCamera const camera{camera_info, optimization::BaCameraState{{pinhole_intrinsics}, Array6d::Zero()},
                                        optimization::BaCameraOptions{false, false}};

    // We use these dummy values here so they are internally consistent in the problem construction.
    uint64_t const timestamp_ns{0};
    AssetId const camera_id{0};

    Frames const frames{{timestamp_ns, {se3_co_w}}};
    optimization::BaObservation const target_observation{camera_id, timestamp_ns, bundle};

    return {{{camera_id, camera}}, frames, {target_observation}};
}

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
    Array3d pinhole_intrinsics;

    if (IsPlane(bundle.points) and bundle.pixels.rows() > 4) {
        auto const dlt_result{Dlt22(bundle)};
        if (not dlt_result) {
            return PnpErrorCode::FailedDlt;  // LCOV_EXCL_LINE
        }

        tf_co_w = *dlt_result;
        pinhole_intrinsics = {1, 0, 0};      // Equivalent to K = I_3x3
        bounds = ImageBounds{-1, 1, -1, 1};  // Unit image dimension bounds
    } else if (bundle.pixels.rows() > 6 and bounds) {
        auto const dlt_result{Dlt23(bundle)};
        if (not dlt_result) {
            return PnpErrorCode::FailedDlt;
        }

        std::tie(tf_co_w, pinhole_intrinsics) = *dlt_result;
    } else {
        return PnpErrorCode::InvalidDlt;
    }

    // TODO(Jack): This is a heuristic slightly hacky looking way to check if the above DLT algorithm evaluation failed.
    //  If we had a better theoretical algorithmic understanding of what causes these failures and how we can detect
    //  them then we could improve this code here.
    Array6d const se3_co_w{geometry::Log(tf_co_w)};
    if (not se3_co_w.allFinite()) {
        return PnpErrorCode::NotAllFinite;  // LCOV_EXCL_LINE
    }

    CameraInfo const camera_info{CameraModel::Pinhole, bounds.value()};
    optimization::BaProblem const ba_problem{BuildPnpBaProblem(camera_info, bundle, pinhole_intrinsics, se3_co_w)};

    auto const result{optimization::BundleAdjustment(ba_problem, 1)};
    if (result.ceres_state.solver_summary.termination_type == ceres::CONVERGENCE) {
        // TODO(Jack): This is a hacky way to get the frame, but the point of the pnp problem construction is that there
        // is only ever one single frame, so we can get away with this here. Does it look nice? No. Is it easy to
        // maintain and understand? No. Some future soul will save us.
        return PoseWithCost{geometry::Exp(result.frames.begin()->second.pose),
                            result.ceres_state.solver_summary.final_cost};
    } else {
        return PnpErrorCode::FailedRefinement;  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::pnp
