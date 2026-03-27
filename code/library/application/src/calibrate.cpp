#include "application/calibrate.hpp"

#include <ranges>

#include "steps/camera_info.hpp"
#include "steps/camera_nonlinear_refinement.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/linear_pose_initialization.hpp"
#include "steps/step_runner.hpp"

#include "utils.hpp"

namespace reprojection::application {

// TODO(Jack): Should we put image_source_signature and image_source into one object? They are 100% related.
void Calibrate(toml::table const& config, ImageSource image_source, std::string const& image_source_signature,
               DbPtr const db) {
    steps::CameraInfoStep const ci_step{image_source_signature, *config["sensor"].as_table(), image_source};
    auto const [camera_info, ci_cache_status]{steps::RunStep<CameraInfo>(ci_step, db)};
    std::cout << "Ci : " << ToString(ci_cache_status) << " " << camera_info.sensor_name << std::endl;

    steps::FeatureExtractionStep const ftext_step{camera_info.sensor_name, image_source_signature, image_source,
                                                  *config["target"].as_table()};
    auto const [targets, ftext_cache_status]{steps::RunStep<CameraMeasurements>(ftext_step, db)};
    std::cout << "Ftext : " << ToString(ftext_cache_status) << " " << std::size(targets) << std::endl;

    steps::IntrinsicInitializationStep const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{steps::RunStep<CameraState>(ii_step, db)};
    std::cout << "Ii : " << ToString(ii_cache_status) << " " << camera_state.intrinsics.transpose() << std::endl;

    steps::LpiStep const lpi_step{camera_info, targets, camera_state};
    auto const [initial_poses, lpi_cache_status]{steps::RunStep<Frames>(lpi_step, db)};
    std::cout << "Lpi : " << ToString(lpi_cache_status) << std::endl;

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};

    steps::CnlrStep const cnlr_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, cnlr_cache_status]{steps::RunStep<OptimizationState>(cnlr_step, db)};
    std::cout << "Cnlr : " << ToString(cnlr_cache_status) << " " << optimized_state.camera_state.intrinsics.transpose()
              << std::endl;
}

}  // namespace reprojection::application
