#include "steps/camera_info.hpp"

#include <toml++/toml.h>

#include "config/config_parse.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "types/config.hpp"

namespace reprojection::steps {

CameraInfoStep::CameraInfoStep(toml::table const& sensor_config, std::shared_ptr<EncodedImages> const& images)
    : images_{images} {
    // TODO(Jack): Instead of parsing the config here inside of the steps themselves we should have one central parsing
    // step. This means we can handle all the errors at one top level instead of throughout the code.
    auto const result{config::Config::Camera::Parse(sensor_config)};
    if (std::holds_alternative<TomlErrorMsg>(result)) {
        throw std::runtime_error{"WE NEED AN ERROR HANDLING STRATEGY!"};
    }

    sensor_name_ = std::get<config::Config::Camera>(result).sensor_name;
    camera_model_ = std::get<config::Config::Camera>(result).camera_model;
}

std::string CameraInfoStep::HashInputs() const { return hashing::HashArguments(sensor_name_, camera_model_, *images_); }

CameraInfo CameraInfoStep::Compute() const {
    if (images_->size() == 0) {
        throw std::runtime_error                                                      // LCOV_EXCL_LINE
            ("we need an error handling strategy for no images to get camera info");  // LCOV_EXCL_LINE
    }

    // Arbitrarily check the size of the first image
    cv::Mat const img{cv::imdecode(images_->begin()->second.data, cv::IMREAD_COLOR)};

    // TOD0(Jack): Is this check really needed? Is it possible that an empty image buffer makes it way here?
    if (img.empty()) {
        throw std::runtime_error                                                        // LCOV_EXCL_LINE
            ("we need an error handling strategy for empty image to get camera info");  // LCOV_EXCL_LINE
    }

    CameraInfo const camera_info{EntityId(),
                                 camera_model_,
                                 {0, static_cast<double>(img.size().width), 0, static_cast<double>(img.size().height)}};

    return camera_info;
}

CameraInfo CameraInfoStep::Load(SqlitePtr const db) const {
    auto const camera_info{database::ReadCameraInfo(db, EntityId())};

    if (not camera_info) {
        throw std::runtime_error("we need a consistent error handling strategy!!!");  // LCOV_EXCL_LINE
    }

    return *camera_info;
}

void CameraInfoStep::Save(CameraInfo const& camera_info, SqlitePtr const db) const {
    database::InsertCameraInfo(db, camera_info);
}

}  // namespace reprojection::steps
