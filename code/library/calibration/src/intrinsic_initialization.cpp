#include "calibration/intrinsic_initialization.hpp"
#include "calibration/linear_pose_initialization.hpp"
#include "projection_functions/intialize_camera.hpp"

#include "parabola_line_initialization.hpp"
#include "utilities.hpp"

namespace reprojection::calibration {

std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                            CameraMeasurements const& targets) {
    auto const [runner, initialization]{SelectInitializationStrategy(CameraModel::DoubleSphere, height, width)};

    double min_error{std::numeric_limits<double>::max()};
    ArrayXd intrinsics;
    for (auto const& target : targets | std::views::values) {
        std::vector<double> const gammas{runner(target)};

        for (auto const gamma_i : gammas) {
            ArrayXd const intrinsics_i{initialization(gamma_i, height, width)};
            ImageBounds const image_bounds{0, width, 0, height};
            auto const camera{projection_functions::InitializeCamera(camera_model, intrinsics_i, image_bounds)};

            auto const result{EstimatePoseViaPinholePnP(camera, target.bundle, image_bounds)};
            if (result.has_value()) {
                auto const [_, final_cost]{*result};
                if (final_cost < min_error) {
                    min_error = final_cost;
                    intrinsics = intrinsics_i;
                }
            }
        }
    }

    return intrinsics;
}

std::pair<CandidateGenerator, IntrinsicsInitializer> SelectInitializationStrategy(CameraModel const camera_model,
                                                                      double const height, double const width) {
    // TOOD ADD VANISHING POINT IN INITIALIZATION!
    CandidateGenerator runner;
    if (camera_model == CameraModel::DoubleSphere or camera_model == CameraModel::UnifiedCameraModel) {
        runner = [height, width](ExtractedTarget const& target) {
            // TODO DO NOT USE VECTOR 2d here! Just pass cx and cy
            return EstimateCandidatesParabolaLine(target, height / 2, width / 2);
        };

    } else {
        throw std::runtime_error("InitializeIntrinsics() 'runner' logic not implemented for: " +
                                 ToString(camera_model));
    }

    IntrinsicsInitializer initializer;
    if (camera_model == CameraModel::DoubleSphere) {
        initializer = projection_functions::DoubleSphere::Initialize;
    } else {
        throw std::runtime_error("InitializeIntrinsics() 'initializer' logic not implemented for: " +
                                 ToString(camera_model));
    }

    return {runner, initializer};
}

// TODO(Jack): Is it really a focal length or a "gamma"?
std::vector<double> EstimateCandidatesParabolaLine(ExtractedTarget const& target, double const cx, double const cy) {
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
