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
            extracted_targets.insert({timestamp_ns, *target});  // LCOV_EXCL_LINE
        }

        // TODO SET AS CONFIG PARAMETER
        bool const gui_enabled{true};
        if (gui_enabled) {
            // TODO COPY AND PASTED FROM FEATURE EXTRACTION DEMO!
            if (target.has_value()) {
                MatrixX2d const& pixels{target->bundle.pixels};
                ArrayX2i const& indices{target->indices};
                for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
                    cv::circle(image, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), 1, cv::Scalar(0, 255, 0), 5,
                               cv::LINE_8);

                    std::string const text{"(" + std::to_string(indices.row(i)[0]) + ", " +
                                           std::to_string(indices.row(i)[1]) + ")"};
                    cv::putText(image, text, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), cv::FONT_HERSHEY_COMPLEX, 0.4,
                                cv::Scalar(255, 255, 255), 1);
                }
            }

            cv::imshow("Tag Detections", image);
            if (cv::waitKey(10) >= 0) {
                break;
            }
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
