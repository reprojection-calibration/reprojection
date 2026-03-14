#include "intrinsic_initialization.hpp"

#include "projection_functions/intialize_camera.hpp"

#include "linear_pose_initialization.hpp"
#include "parabola_line_initialization.hpp"
#include "utilities.hpp"
#include "vanishing_point_initialization.hpp"

namespace reprojection::calibration {

// NOTE(Jack): This method is where runtime configuration meets the compiler. What this means is that every time we add
// new camera models we will also need to come and add them manually here. That is a sign that we are possibly doing
// something wrong. We might be able to achieve the same result automatically (i.e. not needing to manually add another
// condition for each camera model) if we intelligently used some templating/type trait/metaprogramming. I tried it once
// and thought it added too much boilerplate. But if we find that as we add more camera models we have to edit several
// of these "runtime-compile" time borders, then we might want to investigate a solution more closely. For now this gets
// the job done!
std::pair<CandidateGenerator, IntrinsicsInitializer> SelectInitializationStrategy(CameraModel const camera_model,
                                                                                  double const height,
                                                                                  double const width) {
    CandidateGenerator runner;
    if (camera_model == CameraModel::DoubleSphere or camera_model == CameraModel::UnifiedCameraModel) {
        runner = [height, width](ExtractedTarget const& target) {
            return EstimateCandidatesParabolaLine(target, height / 2, width / 2);
        };
    } else if (camera_model == CameraModel::Pinhole) {
        runner = [](ExtractedTarget const& target) { return EstimateCandidatesVanishingPoint(target); };
    } else {
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - InitializeIntrinsics() 'runner' logic not implemented for: " +  // LCOV_EXCL_LINE
            ToString(camera_model));  // LCOV_EXCL_LINE
    }

    IntrinsicsInitializer initializer;
    if (camera_model == CameraModel::Pinhole) {
        initializer = projection_functions::Pinhole::Initialize;
    } else if (camera_model == CameraModel::DoubleSphere) {
        initializer = projection_functions::DoubleSphere::Initialize;
    } else {
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - InitializeIntrinsics() 'initializer' logic not implemented for: " +  // LCOV_EXCL_LINE
            ToString(camera_model));  // LCOV_EXCL_LINE
    }

    return {runner, initializer};
}

// TODO(Jack): Roughly the same point as for EstimateCandidatesVanishingPoint(), do we really need rows AND cols? Or
//  would just one of them suffice? If we get too many candidates then the reprojection error calculation might take a
//  really long time.
std::vector<double> EstimateCandidatesParabolaLine(ExtractedTarget const& target, double const cx, double const cy) {
    auto const [rows, cols]{SortIntoRowsAndCols(target)};

    std::vector<double> gammas;

    auto const estimate_gammas = [cx, cy, &gammas](std::vector<Bundle> const& bundles) {
        for (auto const& bundle : bundles) {
            auto const gamma_i{ParabolaLineInitialization({cx, cy}, bundle.pixels)};
            if (gamma_i.has_value()) {
                gammas.push_back(gamma_i.value());
            }
        }
    };

    estimate_gammas(rows);
    estimate_gammas(cols);

    return gammas;
}

std::vector<double> EstimateCandidatesVanishingPoint(ExtractedTarget const& target) {
    auto const [rows, cols]{SortIntoRowsAndCols(target)};

    // TODO(Jack): Do we really need to pair all rows with all columns? Or would it be enough to just take one row and
    //  pair it with all columns? The current version means we will get a ton of options! More than we need probably.
    std::vector<double> gammas;
    for (auto const& row : rows) {
        for (auto const& col : cols) {
            auto const gamma_i{VanishingPointInitialization(row.pixels, col.pixels)};
            if (gamma_i.has_value()) {
                gammas.push_back(gamma_i.value());
            }
        }
    }

    return gammas;
}

}  // namespace reprojection::calibration
