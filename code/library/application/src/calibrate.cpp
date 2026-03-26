
#include "application/calibrate.hpp"

#include <ranges>

#include "application/step_runner.hpp"
#include "application/steps.hpp"
#include "calibration/initialization_methods.hpp"

namespace reprojection::application {

// TODO MOVE TO FILE
OptimizationState AlignRotations(OptimizationState state) {
    Vector3d so3_i_1{std::cbegin(state.frames)->second.pose.head<3>()};
    for (auto& frame_i : state.frames | std::views::values) {
        Vector3d so3_i{frame_i.pose.head<3>()};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.pose.head<3>() = so3_i;

        so3_i_1 = so3_i;
    }

    return state;
}

// TODO HOW DO WE PASS image_provider? const& or by value or by what?
// TODO HOW SHOULD WE PASS THE CONFIG? SHOULD IT ALREADY BE PARSED?
void Calibrate(toml::table const& config, ImageProvider image_source, std::string_view image_cache_key,
               DbPtr const db) {
    FeatureExtractionStep const ftex_step{image_source, std::string(image_cache_key), *config["target"].as_table(),
                                          *config["sensor"].as_table()};
    auto const [result, ftex_cache_status]{RunStep<FeatureExtractionStep::Result>(ftex_step, db)};
    auto const& [camera_info, targets]{result};

    IntrinsicInitializationStep const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{RunStep<CameraState>(ii_step, db)};

    LpiStep const lpi_step{camera_info, targets, camera_state};
    auto const [initial_poses, lpi_cache_status]{RunStep<Frames>(lpi_step, db)};

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};
    CnlrStep const cnlr_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, cnlr_cache_status]{RunStep<OptimizationState>(cnlr_step, db)};
}

}  // namespace reprojection::application
