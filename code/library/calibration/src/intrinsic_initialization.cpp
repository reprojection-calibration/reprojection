#include "intrinsic_initialization.hpp"

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
    } else if (camera_model == CameraModel::Pinhole or camera_model == CameraModel::PinholeRadtan4) {
        runner = [](ExtractedTarget const& target) { return EstimateCandidatesVanishingPoint(target); };
    } else {
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - InitializeIntrinsics() 'runner' logic not implemented for: " +  // LCOV_EXCL_LINE
            ToString(camera_model));  // LCOV_EXCL_LINE
    }

    IntrinsicsInitializer initializer;
    if (camera_model == CameraModel::DoubleSphere) {
        initializer = projection_functions::DoubleSphere::Initialize;
    } else if (camera_model == CameraModel::Pinhole) {
        initializer = projection_functions::Pinhole::Initialize;
    } else if (camera_model == CameraModel::PinholeRadtan4) {
        initializer = projection_functions::PinholeRadtan4::Initialize;
    } else if (camera_model == CameraModel::UnifiedCameraModel) {
        initializer = projection_functions::UnifiedCameraModel::Initialize;
    } else {
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - InitializeIntrinsics() 'initializer' logic not implemented for: " +  // LCOV_EXCL_LINE
            ToString(camera_model));  // LCOV_EXCL_LINE
    }

    return {runner, initializer};
}

std::vector<double> EstimateCandidatesParabolaLine(ExtractedTarget const& target, double const cx, double const cy) {
    auto const [_, cols]{SortIntoRowsAndCols(target)};

    // NOTE(Jack): We could also iterate over the rows and get the row estimate, but then we roughly double the number
    // of initializations we need to run. Therefore, we arbitrarily limit ourselves to the columns (could also have
    // chosen the rows). If one day we find out that we actually need to check both the rows and columns than of course
    // we can change this here.
    std::vector<double> gammas;
    for (const auto& [pixels, _] : cols) {
        auto const gamma_i{ParabolaLineInitialization({cx, cy}, pixels)};
        if (gamma_i.has_value()) {
            gammas.push_back(*gamma_i);
        }
    }

    return gammas;
}

std::vector<double> EstimateCandidatesVanishingPoint(ExtractedTarget const& target) {
    auto const [rows, cols]{SortIntoRowsAndCols(target)};

    if (std::size(rows) == 0) {
        return {};
    }
    Bundle const row_0{rows[0]};

    // NOTE(Jack): We could have ran every row/col pair, but that means the number of options explodes with target
    // size. Therefore, we arbitrarily match all cols against the first row. We can also match the first column against
    // every row, it is an arbitrary choice, but limiting this here to a single other row speeds things up a lot!
    std::vector<double> gammas;
    for (auto const& col : cols) {
        auto const gamma_i{VanishingPointInitialization(row_0.pixels, col.pixels)};
        if (gamma_i.has_value()) {
            gammas.push_back(*gamma_i);
        }
    }

    return gammas;
}

}  // namespace reprojection::calibration
