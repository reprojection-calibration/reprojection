#include "calibration/focal_length_initialization.hpp"

#include "parabola_line_initialization.hpp"
#include "utilities.hpp"

namespace reprojection::calibration {

std::pair<GammaRunner, FocalLengthInit> RuntimeInitializationDispatch(CameraModel const camera_model,
                                                                      double const height, double const width) {
    // TOOD ADD VANISHING POINT IN INITIALIZATION!
    GammaRunner runner;
    if (camera_model == CameraModel::DoubleSphere or camera_model == CameraModel::UnifiedCameraModel) {
        runner = [height, width](ExtractedTarget const& target) {
            // TODO DO NOT USE VECTOR 2d here! Just pass cx and cy
            return InitializeFocalLengthParabolaLine(target, height / 2, width / 2);
        };

    } else {
        throw std::runtime_error("InitializeIntrinsics() 'runner' logic not implemented for: " +
                                 ToString(camera_model));
    }

    FocalLengthInit initializer;
    if (camera_model == CameraModel::DoubleSphere) {
        initializer = projection_functions::DoubleSphere::Initialize;
    } else {
        throw std::runtime_error("InitializeIntrinsics() 'initializer' logic not implemented for: " +
                                 ToString(camera_model));
    }

    return {runner, initializer};
}

// TODO(Jack): Is it really a focal length or a "gamma"?
std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, double const cx, double const cy) {
    auto const [rows, cols]{SortIntoRowsAndCols(target)};

    std::vector<double> focal_lengths;

    auto const collect = [cx, cy, &focal_lengths](std::vector<Bundle> const& bundles) {
        for (auto const& bundle : bundles) {
            auto const f{ParabolaLineInitialization({cx, cy}, bundle.pixels)};
            if (f.has_value()) {
                focal_lengths.push_back(f.value());
            }
        }
    };

    collect(rows);
    collect(cols);

    return focal_lengths;
}

}  // namespace reprojection::calibration
