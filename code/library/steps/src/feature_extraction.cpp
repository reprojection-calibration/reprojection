#include "steps/feature_extraction.hpp"

#include "caching/cache_keys.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "feature_extraction/target_extraction.hpp"

namespace reprojection::application {

std::string FeatureExtractionStep::CacheKey() const { return cache_key; }

FeatureExtractionStep::Result FeatureExtractionStep::Compute() const {
    // TODO(Jack): Is it really appropriate to use a toml table here instead of a struct?
    auto const extractor{feature_extraction::CreateTargetExtractor(target_config)};

    CameraMeasurements extracted_targets;
    cv::Size size;
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, image]{*data};
        // TODO(Jack): Does the format of the image matter? gray or rgb etc?
        std::optional<ExtractedTarget> const target{extractor->Extract(image)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, target.value()});
        }

        // TODO(Jack): We only need to get the size of one image, but as the only way we access the images is in this
        // loop, so we need to put the code in the loop...
        size = image.size();
    }

    // TODO TIDY UP!
    CameraInfo const camera_info{SensorName(),
                                 ToCameraModel(sensor_config["camera_model"].as_string()->get()),
                                 {0, static_cast<double>(size.width), 0, static_cast<double>(size.height)}};

    return {camera_info, extracted_targets};
}

FeatureExtractionStep::Result FeatureExtractionStep::Load(
    std::shared_ptr<database::CalibrationDatabase const> const db) const {
    auto const camera_info{database::ReadCameraInfo(db, SensorName())};

    if (not camera_info) {
        throw std::runtime_error("we need a consistent error handling strategy!!!");
    }

    return {*camera_info, database::GetExtractedTargetData(db, SensorName())};
}

void FeatureExtractionStep::Save(Result const& result, std::shared_ptr<database::CalibrationDatabase> const db) const {
    auto const& [camera_info, extracted_targets]{result};
    database::WriteToDb(camera_info, db);
    database::WriteToDb(extracted_targets, SensorName(), db);
}

}  // namespace reprojection::application
