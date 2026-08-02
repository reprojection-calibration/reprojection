#include "steps/camera_info.hpp"

#include "hashing/hashing.hpp"

namespace reprojection::steps {

CameraInfoStep::CameraInfoStep(AssetId const camera_id, StepId const image_loading_id, CameraModel const camera_model,
                               database::CalibrationDatabase& db)
    : camera_id_{camera_id},
      camera_model_{camera_model},
      images_{std::make_shared<EncodedImages>(db.ImagesSelect(image_loading_id, camera_id))} {}

Hash CameraInfoStep::CacheKey() const {
    return hashing::HashArguments(camera_model_, *images_);
}

void CameraInfoStep::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    // TODO(Jack): Is this really a good error handling strategy? What does this look like to the user/application side?
    // Should this be checked in the constructor? Problem with that is that we cant artificially trigger a cache hit
    // then.
    if (std::size(*images_) == 0) {
        throw std::runtime_error(
            std::format("Camera info step called with no images loaded - "
                        "step id {}, camera model {}",
                        step_id.value, ToString(camera_model_)));
    }

    // Arbitrarily check the size of the first image - in the constructor we already checked to make sure that there is
    // at least one image so this should be safe.
    cv::Mat const img{cv::imdecode(images_->begin()->second.data, cv::IMREAD_COLOR)};
    // TOD0(Jack): Can we have a more uniform and traceable error handling strategy? Here and in the constructor.
    if (img.empty()) {
        throw std::runtime_error("Attempted to decode image in CameraInfoStep, result was empty! ");
    }

    CameraInfo const camera_info{camera_model_,
                                 {0, static_cast<double>(img.size().width), 0, static_cast<double>(img.size().height)}};

    db.CameraInfoInsert(step_id, camera_id_, camera_info);
}

}  // namespace reprojection::steps
