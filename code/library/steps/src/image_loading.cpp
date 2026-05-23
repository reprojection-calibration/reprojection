#include "steps/image_loading.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

// TODO(Jack): The name of the class variable "cache_key" is misleading because it is not a cache key but really a
// serialized data signature. We should fix this name to clarify its purpose and use.
std::string ImageLoadingStep::CacheKey() const { return caching::CacheKey(cache_key); }

std::shared_ptr<EncodedImages> ImageLoadingStep::Compute() const {
    auto encoded_images = std::make_shared<EncodedImages>();
    int num_images{0};
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, img]{*data};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed for " + std::string(sensor_name));  // LCOV_EXCL_LINE
        }

        encoded_images->insert({timestamp_ns, ImageBuffer{buffer}});

        ++num_images;
        if (num_images % 50 == 0) {
            log->debug("{{'step': '{}', 'stage': '{}', 'sensor_id': '{}', 'num_images': {}}}",  // LCOV_EXCL_LINE
                       ToString(step_type), "Compute()", SensorName(), num_images);             // LCOV_EXCL_LINE
        }
    }

    return encoded_images;
}  // LCOV_EXCL_LINE

std::shared_ptr<EncodedImages> ImageLoadingStep::Load(SqlitePtr const db) const {
    return std::make_shared<EncodedImages>(database::ReadEncodedImages(db, SensorName()));
}

void ImageLoadingStep::Save(std::shared_ptr<EncodedImages const> const encoded_images, SqlitePtr const db) const {
    database::WriteToDb(*encoded_images, SensorName(), db);
}

}  // namespace reprojection::steps
