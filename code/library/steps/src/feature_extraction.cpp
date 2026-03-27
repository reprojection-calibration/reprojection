#include "steps/feature_extraction.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "feature_extraction/target_extraction.hpp"

namespace reprojection::application {

// ERROR(Jack): The feature extraction here is in the unique place where it control two things, the extracted features
// and the camera info. Right now we are only caching here on the input images, which means that as long as the images
// did not change we will get a cache hit, but what happens if the camera info config change? Then nothing will happen,
// this needs to be fixed!
std::string FeatureExtractionStep::CacheKey() const { return caching::CacheKey(cache_key); }

CameraMeasurements FeatureExtractionStep::Compute() const {
    // TODO(Jack): Is it really appropriate to use a toml table here instead of a struct?
    auto const extractor{feature_extraction::CreateTargetExtractor(target_config)};

    CameraMeasurements extracted_targets;
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, image]{*data};

        std::optional<ExtractedTarget> const target{extractor->Extract(image)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, target.value()});
        }
    }

    return extracted_targets;
}

CameraMeasurements FeatureExtractionStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    return database::GetExtractedTargetData(db, SensorName());
}

void FeatureExtractionStep::Save(CameraMeasurements const& extracted_targets,
                                 std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(extracted_targets, SensorName(), db);
}

}  // namespace reprojection::application
