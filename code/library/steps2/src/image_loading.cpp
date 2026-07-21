#include "steps/image_loading.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

ImageLoading::ImageLoading(AssetId const camera_id, std::string_view serialized_image_sampler,
                           ImageSampler const& image_sampler)
    : camera_id_{camera_id},
      cache_key_{hashing::HashArguments(serialized_image_sampler)},
      image_sampler_{image_sampler} {}

Hash ImageLoading::CacheKey(database::CalibrationDatabase& db) {
    // NOTE(Jack): The image loading is unique in that it does not load anythign from the database but instead
    // bootstraps directly from the user/application input.
    static_cast<void>(db);

    return cache_key_;
}

void ImageLoading::Execute(database::CalibrationDatabase& db, StepId const step_id) {
    auto encoded_images = std::make_shared<EncodedImages>();
    int num_images{0};
    while (auto const data{image_source_()}) {
        auto const& [timestamp_ns, img]{*data};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed for " + std::string(camera_name_));  // LCOV_EXCL_LINE
        }

        encoded_images->insert({timestamp_ns, ImageBuffer{buffer}});

        ++num_images;
        if (num_images % 50 == 0) {
            log->debug("{{'step': '{}', 'stage': '{}', 'sensor_id': '{}', 'num_images': {}}}",  // LCOV_EXCL_LINE
                       ToString(StepType()), "Compute()", EntityId(), num_images);              // LCOV_EXCL_LINE
        }
    }

    db.ImagesInsert(step_id, camera_id_, encoded_images);
}


}  // namespace reprojection::steps
