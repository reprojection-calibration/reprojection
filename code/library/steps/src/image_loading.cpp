#include "steps/image_loading.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

// TODO(Jack): The name of the class variable "cache_key" is misleading because it is not a cache key but really a
// serialized data signature. We should fix this name to clarify its purpose and use.
std::string ImageLoadingStep::CacheKey() const { return caching::CacheKey(cache_key); }

EncodedImages ImageLoadingStep::Compute() const {
    EncodedImages encoded_images;
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, img]{*data};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed for " + std::string(sensor_name));  // LCOV_EXCL_LINE
        }

        encoded_images.insert({timestamp_ns, ImageBuffer{buffer}});  // LCOV_EXCL_LINE
    }

    return encoded_images;
}  // LCOV_EXCL_LINE

EncodedImages ImageLoadingStep::Load(SqlitePtr const db) const { return database::GetEncodedImages(db, SensorName()); }

void ImageLoadingStep::Save(EncodedImages const& encoded_images, SqlitePtr const db) const {
    database::WriteToDb(encoded_images, SensorName(), db);
}

}  // namespace reprojection::steps
