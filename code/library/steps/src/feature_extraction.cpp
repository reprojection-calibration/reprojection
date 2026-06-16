#include "steps/feature_extraction.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "feature_extraction/target_extraction.hpp"
#include "hashing/hashing.hpp"
#include "image_viewer/image_viewer.hpp"

namespace reprojection::steps {

std::string FeatureExtraction::HashInputs() const {
    std::ostringstream oss;
    oss << show_extraction_;

    return hashing::HashArguments(target_info_, *images_, oss.str());
}

// TODO(Jack): We really need to split the visualization logic from the core computation!
// NOTE(Jack): The unit tests and CI pipeline run headless which means that we cannot get the GUI show feature
// extraction code path unit tested and covered.
CameraMeasurements FeatureExtraction::Compute() const {
    // TODO(Jack): Is it really appropriate to use a toml table here instead of a struct?
    auto const extractor{feature_extraction::CreateTargetExtractor(target_info_)};

    CameraMeasurements extracted_targets;
    for (auto const& [timestamp_ns, buffer] : *images_) {
        // TODO COPY AND PASTED FROM CAMERA INFO AND THE STEPS TEST!
        cv::Mat const img{cv::imdecode(buffer.data, cv::IMREAD_UNCHANGED)};
        if (img.empty()) {
            throw std::runtime_error(
                "we need an error handling strategy for empty images in feature extraction");  // LCOV_EXCL_LINE
        }

        std::optional<ExtractedTarget> const target{extractor->Extract(img)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, *target});  // LCOV_EXCL_LINE
        }

        // NOTE(Jack): If we have an extracted target then draw the points and display. Otherwise, just display the
        // image.
        if (show_extraction_) {
            if (target.has_value()) {                          // LCOV_EXCL_LINE
                feature_extraction::DrawTarget(*target, img);  // LCOV_EXCL_LINE
            }

            // TODO(Jack): Here we are giving the GUI image displayer the possibility to end the feature extraction, is
            // that really an interaction/power we want this code to have?
            // TODO(Jack): Right now if the user requests showing the extraction but there is no available GUI we will
            // just crash here. We might want to wrap the window visualizer in a little class with a factory function,
            // and then log to the user a warning if they requested visualization but here is no gui device.
            static image_viewer::ImageViewer viewer(
                std::make_unique<image_viewer::OpenCvGuiInterface>("Target Feature Extraction"),  // LCOV_EXCL_LINE
                std::make_unique<image_viewer::OpenCvKeyboardInput>());                           // LCOV_EXCL_LINE
            viewer.Show(img);                                                                     // LCOV_EXCL_LINE
            if (viewer.ShouldQuit()) {                                                            // LCOV_EXCL_LINE
                break;                                                                            // LCOV_EXCL_LINE
            }
        }
    }

    return extracted_targets;
}

CameraMeasurements FeatureExtraction::Load(SqlitePtr const db) const { return database::ReadTargets(db, EntityId()); }

void FeatureExtraction::Save(CameraMeasurements const& extracted_targets, SqlitePtr const db) const {
    database::InsertTargets(db, EntityId(), extracted_targets);
}

}  // namespace reprojection::steps
