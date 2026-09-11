#include "steps/camera_info.hpp"

#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

CameraInfoStep::CameraInfoStep(AssetId const camera_id, StepId const image_loading_id, CameraModel const camera_model,
                               SqlitePtr const db)
    : camera_id_{camera_id},
      camera_model_{camera_model},
      images_{std::make_shared<ImageSamples>(database::ImagesSelect(db.get(), image_loading_id, camera_id))} {}

Hash CameraInfoStep::CacheKey() const {
    // NOTE(Jack): See FeatureExtraction::CacheKey() comment as to why we need the camera asset id.
    return hashing::HashArguments(camera_id_.value, camera_model_, *images_);
}

void CameraInfoStep::Execute(StepId const step_id, SqlitePtr const db) const {
    // TODO(Jack): Should this be checked in the constructor? Problem with that is that we cant artificially trigger a
    // cache hit then. But it seems like if we can already know this is a problem then that we should not let
    // construction finish.
    if (std::size(*images_) == 0) {
        // LCOV_EXCL_START
        log->error("{{{}, 'msg': 'No images loaded.'}}", StepLogInfo{Type(), step_id, camera_id_},
                   ToString(camera_model_));
        std::exit(1);
        // LCOV_EXCL_STOP
    }

    // Check the size of the first image to get the image dimensions.
    cv::Mat const img{cv::imdecode(images_->begin()->second.data, cv::IMREAD_COLOR)};
    if (img.empty()) {
        // LCOV_EXCL_START
        log->error("{{{}, 'msg': 'Attempted to decode image but result was empty.'}}",
                   StepLogInfo{Type(), step_id, camera_id_}, ToString(camera_model_));
        std::exit(1);
        // LCOV_EXCL_STOP
    }

    CameraInfo const camera_info{camera_model_,
                                 {0, static_cast<double>(img.size().width), 0, static_cast<double>(img.size().height)}};

    log->info("{{{}, 'camera_info': {{'camera_model': {}, 'height': {}, 'width': {}}}}}",
              StepLogInfo{Type(), step_id, camera_id_}, ToString(camera_model_), camera_info.bounds.v_max,
              camera_info.bounds.u_max);

    database::CameraInfoInsert(db.get(), step_id, camera_id_, camera_info);
}

}  // namespace reprojection::steps
