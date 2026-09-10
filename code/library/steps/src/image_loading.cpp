#include "steps/image_loading.hpp"

#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
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

void ImageLoading::Execute(StepId const step_id, SqlitePtr const db) const {
    auto encoded_images = std::make_shared<ImageSamples>();
    int num_images{0};
    while (auto const data{image_sampler_()}) {
        auto const& [timestamp_ns, img]{*data};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            // LCOV_EXCL_START
            log->error("{{{}, 'msg': 'cv::imencode() failed at timestamp_ns {}.'}}",
                       StepLogInfo{Type(), step_id, camera_id_}, timestamp_ns);
            std::exit(1);
            // LCOV_EXCL_STOP
        }

        encoded_images->insert({timestamp_ns, ImageBuffer{buffer}});

        ++num_images;
        if (num_images % 50 == 0) {
            // LCOV_EXCL_START
            log->debug("{{{}, 'num_images': {}}}", StepLogInfo{Type(), step_id, camera_id_}, num_images);
            // LCOV_EXCL_STOP
        }
    }

    log->info("{{{}, 'num_images': {}}}", StepLogInfo{Type(), step_id, camera_id_}, std::size(*encoded_images));

    database::ImagesInsert(db.get(), step_id, camera_id_, *encoded_images);
}

}  // namespace reprojection::steps
