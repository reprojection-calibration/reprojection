#include "steps/feature_extraction.hpp"

#include <ranges>

#include "database/calib_db.hpp"
#include "feature_extraction/target_extraction.hpp"
#include "hashing/hashing.hpp"
#include "image_viewer/image_viewer.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

FeatureExtraction::FeatureExtraction(AssetId const camera_id, std::string_view serialized_image_sampler,
                                     ImageSampler const& image_sampler, bool const show_extraction,
                                     StepId const target_info_id, AssetId const target_id, SqlitePtr const db)
    : camera_id_{camera_id},
      image_sampler_hash_{hashing::HashArgs(serialized_image_sampler)},
      image_sampler_{image_sampler},
      show_extraction_{show_extraction},
      target_info_{ValueOrExit(database::TargetInfoSelect(db.get(), target_info_id, target_id), log)} {}

Hash FeatureExtraction::CacheKey() const {
    // TODO(Jack): We should not strictly need the camera_id_ here as part of they key because the target info and
    // images_ should uniquely identify the feature extraction. However a problem arises when we have artifically
    // triggered cache hits (for example in the benchmark testing) Where the images_ are empty and that causes the
    // cache key to no longer be unique across different cameras. To prevent this we added the asset id. If this is
    // really a good way to solve this is unclear. The problem I see is that the asset id is not some universal
    // "forever" identifier, and therefore its use here seems like it might causes problems down the line.
    return hashing::HashArgs(camera_id_.value, image_sampler_hash_.value, target_info_);
}

// TODO(Jack): We really need to split the visualization logic from the core computation!
// NOTE(Jack): The unit tests and CI pipeline run headless which means that we cannot get the GUI show feature
// extraction code path unit tested and covered.
void FeatureExtraction::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const extractor{feature_extraction::CreateTargetExtractor(target_info_)};

    // TODO LOG HOW MANY SAMPLES HAVE BEEN RUN!
    TargetSamples extracted_targets;
    while (auto const data{image_sampler_()}) {
        auto const& [timestamp_ns, img]{*data};

        std::optional const target{extractor->Extract(img)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, *target});  // LCOV_EXCL_LINE
        }

        // LCOV_EXCL_START
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
        // LCOV_EXCL_STOP

        // TODO(Jack): Given the current foreign key constraints we need to insert the images into the image table here.
        // Because we construct the image samples here with an empty buffer the sqlite table will just get a null entry.
        // TODO(Jack): Do we just need to completely refactor to replace the role in the FK tree that images play with
        // the extracted targets?
        ImageSamples const imgs{[&extracted_targets] {
            ImageSamples data;
            for (auto const& timestamp_ns : extracted_targets | std::views::keys) {
                data.insert({timestamp_ns, {}});
            }
            return data;
        }()};
        database::ImagesInsert(db.get(), step_id, camera_id_, imgs);

        // WARN(Jack): Originall the targets had a FK relationship on a seperate now non-existent image loading step
        // which is why we now pass the source_step_id as our current step instead of the image loading step id which no
        // longer exists.
        // TODO(Jack): Can we remove the FK dep on the image loading step considering now that the feature extraction
        // step is also what delineates the loaded images?
        database::TargetsInsert(db.get(), step_id, step_id, camera_id_, extracted_targets);
    }
}

}  // namespace reprojection::steps
