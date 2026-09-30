#include "steps/camera_info.hpp"

#include "database/calib_db.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

CameraInfoStep::CameraInfoStep(AssetId const camera_id, std::string_view serialized_image_sampler,
                               ImageSampler const& image_sampler, CameraModel const camera_model)
    : camera_id_{camera_id},
      image_sampler_hash_{hashing::HashArgs(serialized_image_sampler)},
      image_sampler_{image_sampler},
      camera_model_{camera_model} {}

Hash CameraInfoStep::CacheKey() const {
    // NOTE(Jack): See FeatureExtraction::CacheKey() comment as to why we need the camera asset id.
    return hashing::HashArgs(camera_id_.value, image_sampler_hash_.value, camera_model_);
}

void CameraInfoStep::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const sample{image_sampler_()};
    if (not sample) {
        // LCOV_EXCL_START
        log->error("{{{}, 'msg': 'No images available.'}}", StepLogInfo{Type(), step_id, camera_id_},
                   ToString(camera_model_));
        std::exit(1);
        // LCOV_EXCL_STOP
    }

    auto const& [timestamp_ns, img]{*sample};
    CameraInfo const camera_info{camera_model_,
                                 {0, static_cast<double>(img.size().width), 0, static_cast<double>(img.size().height)}};

    database::CameraInfoInsert(db.get(), step_id, camera_id_, camera_info);

    log->info("{{{}, 'camera_info': {{'camera_model': {}, 'height': {}, 'width': {}}}}}",
              StepLogInfo{Type(), step_id, camera_id_}, ToString(camera_model_), camera_info.bounds.v_max,
              camera_info.bounds.u_max);

    // TODO(Jack): With this one stored sample can we do anything smart like allows reruns even when the source data is
    // gone? We should at least offer the user the chance to view this image in the dashboard.
    // NOTE(Jack): We take this one image and write it to the database. This is a little arbitrary but it is nice
    // to at least have one image in the calibration record.
    std::vector<uchar> buffer;
    if (not cv::imencode(".png", img, buffer)) {
        // LCOV_EXCL_START
        log->error("{{{}, 'msg': 'cv::imencode() failed at timestamp_ns {}.'}}",
                   StepLogInfo{Type(), step_id, camera_id_}, timestamp_ns);
        std::exit(1);
        // LCOV_EXCL_STOP
    }

    database::ImagesInsert(db.get(), step_id, camera_id_, ImageSamples{{timestamp_ns, ImageBuffer{buffer}}});
}

}  // namespace reprojection::steps
