#include "steps/feature_extraction.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "feature_extraction/target_extraction.hpp"

namespace reprojection::steps {

std::string FeatureExtractionStep::CacheKey() const {
    std::ostringstream oss;
    oss << target_config;

    return caching::CacheKey(cache_key + oss.str());
}

// TODO(Jack): We really need to split the visualization logic from the core computation!
// NOTE(Jack): The unit tests and CI pipeline run headless which means that we cannot get the GUI show feature
// extraction code path unit tested and covered.
CameraMeasurements FeatureExtractionStep::Compute() const {
    // TODO(Jack): Processing the toml config here is not so pretty! But we do not have a final strategy for dealing
    // with the config tables and the feature extraction code.
    bool show_extraction{false};  // Sensible default
    if (auto const node{target_config["show_extraction"]}) {
        show_extraction = node.as_boolean()->get();  // LCOV_EXCL_LINE
    }

    // TODO(Jack): Right now if the user requests showing the extraction but there is no availalbe GUI we will just
    // crash here. We might want to wrap the window visualizer in a little class with a factory function, and then log
    // to the user a warning if they requested visualization but here is no gui device.
    if (show_extraction) {
        cv::namedWindow("Target Extraction", cv::WINDOW_AUTOSIZE);  // LCOV_EXCL_LINE
    }

    // TODO(Jack): Is it really appropriate to use a toml table here instead of a struct?
    auto const extractor{feature_extraction::CreateTargetExtractor(target_config)};

    CameraMeasurements extracted_targets;
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, img]{*data};

        std::optional<ExtractedTarget> const target{extractor->Extract(img)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, *target});  // LCOV_EXCL_LINE
        }

        // NOTE(Jack): If we have an extracted target then draw the points and display. Otherwise, just display the
        // image.
        if (show_extraction) {
            if (target.has_value()) {                          // LCOV_EXCL_LINE
                feature_extraction::DrawTarget(*target, img);  // LCOV_EXCL_LINE
            }

            cv::imshow("Target Extraction", img);  // LCOV_EXCL_LINE
            // TODO(Jack): Should we make the delay time here configurable?
            cv::waitKey(5);  // LCOV_EXCL_LINE
        }
    }

    return extracted_targets;
}

CameraMeasurements FeatureExtractionStep::Load(SqlitePtr const db) const {
    return database::GetExtractedTargetData(db, SensorName());
}

void FeatureExtractionStep::Save(CameraMeasurements const& extracted_targets, SqlitePtr const db) const {
    database::WriteToDb(extracted_targets, SensorName(), db);
}

}  // namespace reprojection::steps
