#include "steps/feature_extraction.hpp"

#include "feature_extraction/target_extraction.hpp"
#include "hashing/hashing.hpp"
#include "image_viewer/image_viewer.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

FeatureExtraction::FeatureExtraction(AssetId const camera_id, StepId const image_loading_id, bool const show_extraction,
                                     StepId const target_info_id, AssetId const target_id,
                                     database::CalibrationDatabase& db)
    : camera_id_{camera_id}, image_loading_id_{image_loading_id}, show_extraction_{show_extraction} {
    auto const target_info_opt{db.TargetInfoSelect(target_info_id, target_id)};
    if (not target_info_opt) {
        log->error(
            "{{'target_info_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to load target info but result was "
            "empty.'}}",
            target_info_id.value, target_id.value);
    }
    target_info_ = *target_info_opt;

    images_ = std::make_shared<EncodedImages>(db.ImagesSelect(image_loading_id, camera_id));
}

Hash FeatureExtraction::CacheKey() const {
    // TODO(Jack): We should not strictly need the camera_id_ here as part of they key because the target info and
    // images_ should uniquely identify the feature extraction. However a problem arises when we have artifically
    // triggered cache hits (for example in the benchmark testing) Where the images_ is empty and that means for the
    // cache key is no longer unique across different cameras. To prevent this we added the EntityId(). If this is
    // really a good way to solve this is unclear. But right now it solves our problem and does cause any new ones :)
    return hashing::HashArguments(camera_id_.value, show_extraction_, target_info_, *images_);
}

// TODO(Jack): We really need to split the visualization logic from the core computation!
// NOTE(Jack): The unit tests and CI pipeline run headless which means that we cannot get the GUI show feature
// extraction code path unit tested and covered.
void FeatureExtraction::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    auto const extractor{feature_extraction::CreateTargetExtractor(target_info_)};

    CameraMeasurements extracted_targets;
    for (auto const& [timestamp_ns, buffer] : *images_) {
        cv::Mat const img{cv::imdecode(buffer.data, cv::IMREAD_UNCHANGED)};
        if (img.empty()) {
            log->error(
                "{{'step_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to decode image but result was empty.'}}",
                step_id.value, camera_id_.value);
        }

        std::optional<ExtractedTarget> const target{extractor->Extract(img)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, *target});
        }

        if (show_extraction_) {
            if (target.has_value()) {
                feature_extraction::DrawTarget(*target, img);
            }

            // TODO(Jack): Here we are giving the GUI image displayer the possibility to end the feature extraction, is
            // that really an interaction/power we want this code to have?
            // TODO(Jack): Right now if the user requests showing the extraction but there is no available GUI we will
            // just crash here. We might want to wrap the window visualizer in a little class with a factory function,
            // and then log to the user a warning if they requested visualization but here is no gui device.
            static image_viewer::ImageViewer viewer(
                std::make_unique<image_viewer::OpenCvGuiInterface>("Target Feature Extraction"),
                std::make_unique<image_viewer::OpenCvKeyboardInput>());

            viewer.Show(img);
            if (viewer.ShouldQuit()) {
                break;
            }
        }
    }

    db.ExtractedTargetsInsert(step_id, image_loading_id_, camera_id_, extracted_targets);
}

}  // namespace reprojection::steps
