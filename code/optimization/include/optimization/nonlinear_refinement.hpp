#pragma once

#include <ceres/ceres.h>

#include <tuple>
#include <vector>

#include "spline/r3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// NOTE(Jack): We are hardcoding that fact that the intrinsics are the same for all cameras! I.e. not that every image
// could have another camera.
std::tuple<std::vector<Isometry3d>, ArrayXd, double> CameraNonlinearRefinement(std::vector<Frame> const& frames,
                                                                               CameraModel const& camera_type,
                                                                               ArrayXd const& intrinsics);

// TODO(Jack): For now should we make this part of the R3SplineNonlinearRefinement namespace?
struct R3Measurement {
    std::uint64_t t_ns;
    Vector3d r3;
    spline::DerivativeOrder type;
};

// NOTE(Jack): At this time it is still not entirely clear if we need to solve this type of problem at all, however this
// code serves as a learning base and step on the way to full pose spline optimization.
// TODO(Jack): Do we really need a class here or can we make it a pass through function? There is no very strong reason
// for state expect that maybe we can better handle invalid constraint input. But that is not clear yet.
class R3SplineNonlinearRefinement {
   public:
    explicit R3SplineNonlinearRefinement(spline::R3SplineState const& spline);

    // NOTE(Jack): We will keep this as no discard because I want to force the user to responsibly handle invalid
    // conditions when adding constraints.
    [[nodiscard]] bool AddConstraint(R3Measurement const& constraint);

    // TODO(Jack): There is no protection which would prevent the user from calling this on an invalid problem (ex. they
    // forgot to add any data). We need to codify the real long term usage strategy here, this is not the final answer!
    ceres::Solver::Summary Solve();

    spline::R3SplineState GetSpline() const;

   private:
    spline::R3SplineState spline_;  // Stores the state we are optimizing
    ceres::Problem problem_;
};

}  // namespace  reprojection::optimization
