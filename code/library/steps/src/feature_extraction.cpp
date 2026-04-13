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

CameraMeasurements FeatureExtractionStep::Compute() const {
    // TODO(Jack): Is it really appropriate to use a toml table here instead of a struct?
    auto const extractor{feature_extraction::CreateTargetExtractor(target_config)};

    CameraMeasurements extracted_targets;
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, image]{*data};

        std::optional<ExtractedTarget> const target{extractor->Extract(image)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, target.value()});  // LCOV_EXCL_LINE
        }
    }

    return extracted_targets;
}

CameraMeasurements FeatureExtractionStep::Load(SqlitePtr const& db) const {
    return database::GetExtractedTargetData(db, SensorName());
}

void FeatureExtractionStep::Save(CameraMeasurements const& extracted_targets,
                                 SqlitePtr const& db) const {
    database::WriteToDb(extracted_targets, SensorName(), db);
}

}  // namespace reprojection::steps
