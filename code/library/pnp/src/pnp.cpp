#include "pnp/pnp.hpp"

#include "calibration_data_views/optimization_view.hpp"
#include "dlt.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "plane_utilities.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::pnp {

// TODO(Jack): What is the canonical way to deal with the camera matrix? I want the pnp algo to know nothing about K if
// possible, because the more we let information about the camera and camera models percolate through the code base, the
// more combined everything will get. That being said, should we specifically pass the pixels here in image coordinates?
// That seems to be the only reasonable coordinate system given how people interact with pixels. Then the next question
// I have is should we normalize the points at the top level pnp function and then feed those both to the DLT and the
// nonlinear refinement? Right now I do not understand what coordinate context to do the nonlinear refinement in.
PnpResult Pnp(Bundle const& bundle) {
    // TODO(Jack): Design this error out of existence by requiring that bundle have matching correspondence!
    if (not(bundle.pixels.rows() == bundle.points.rows())) {
        return PnpStatusCode::MismatchedCorrespondences;
    }

    Isometry3d tf;
    Array4d pinhole_intrinsics;
    if (IsPlane(bundle.points) and bundle.pixels.rows() > 4) {
        tf = Dlt22(bundle);
        pinhole_intrinsics = {1, 1, 0, 0};  // Equivalent to K = I_3x3
    } else if (bundle.pixels.rows() > 6) {
        std::tie(tf, pinhole_intrinsics) = Dlt23(bundle);
    } else {
        return PnpStatusCode::NotEnoughPoints;
    }

    // TODO(Jack): Can we check directly the matrix if it has nans? Or do we need to do this conversion?
    // TODO(Jack): Is there a more sound theoretical way to check and describe this failure?
    // TODO(Jack): Is it possible that nans get introduced in the nonlinear step that comes next? If so then we should
    // move this check to right before the return.
    // TOOD(Jack): Add a test that triggers this condition!
    Array6d const se3{geometry::Log(tf)};
    if (se3.hasNaN()) {
        return PnpStatusCode::ContainsNans;
    }

    CameraCalibrationData data{
        {"", CameraModel::Pinhole, {0, 720, 0, 480}}, pinhole_intrinsics, {}, {{0, {{bundle, {}}, geometry::Log(tf)}}}};
    optimization::CameraNonlinearRefinement(OptimizationDataView(data));

    // There is only one single frame here, therefore we can simply use std::cbegin to access the result.
    return geometry::Exp(std::cbegin(data.frames)->second.optimized_pose);
}

}  // namespace reprojection::pnp