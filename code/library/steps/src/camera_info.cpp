#include "steps/camera_info.hpp"

#include <toml++/toml.h>

#include "hashing/hashing.hpp"
#include "config/config_parsing.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

CameraInfoStep::CameraInfoStep(toml::table const& _sensor_config, std::shared_ptr<EncodedImages> const& _images)
    : images{_images} {
    std::tie(sensor_name, camera_model) = config::ParseSensorConfig(_sensor_config);
}

std::string CameraInfoStep::CacheKey() const { return hashing::HashArguments(sensor_name, camera_model, *images); }

CameraInfo CameraInfoStep::Compute() const {
    if (images->size() == 0) {
        throw std::runtime_error(
            "we need an error handling strategy for no images to get camera info");  // LCOV_EXCL_LINE
    }

    // Arbitrarily check the size of the first image
    cv::Mat const img{cv::imdecode(images->begin()->second.data, cv::IMREAD_COLOR)};

    // TOD0(Jack): Is this check really needed? Is it possible that an empty image buffer makes it way here?
    if (img.empty()) {
        throw std::runtime_error(
            "we need an error handling strategy for empty image to get camera info");  // LCOV_EXCL_LINE
    }

    CameraInfo const camera_info{SensorName(),
                                 camera_model,
                                 {0, static_cast<double>(img.size().width), 0, static_cast<double>(img.size().height)}};

    return camera_info;
}

CameraInfo CameraInfoStep::Load(SqlitePtr const db) const {
    auto const camera_info{database::ReadCameraInfo(db, SensorName())};

    if (not camera_info) {
        throw std::runtime_error("we need a consistent error handling strategy!!!");  // LCOV_EXCL_LINE
    }

    return *camera_info;
}

void CameraInfoStep::Save(CameraInfo const& camera_info, SqlitePtr const db) const {
    database::InsertCameraInfo(db, camera_info);
}

}  // namespace reprojection::steps
