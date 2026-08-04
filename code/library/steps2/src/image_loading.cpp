#include "steps/image_loading.hpp"

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

Hash ImageLoading::CacheKey() const { return cache_key_; }

void ImageLoading::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    auto encoded_images = std::make_shared<EncodedImages>();
    int num_images{0};
    while (auto const data{image_sampler_()}) {
        auto const& [timestamp_ns, img]{*data};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            log->error("{{'step_id': {}, 'asset_id': {}, 'msg': 'cv::imencode() failed at timestamp_ns {}.'}}",
                       step_id.value, camera_id_.value, timestamp_ns);
            std::exit(1);
        }

        encoded_images->insert({timestamp_ns, ImageBuffer{buffer}});

        ++num_images;
        if (num_images % 50 == 0) {
            log->debug("{{'step_id': {}, 'asset_id': {}, 'num_images': {}}}", step_id.value, camera_id_.value,
                       num_images);
        }
    }

    db.ImagesInsert(step_id, camera_id_, *encoded_images);
}

}  // namespace reprojection::steps
