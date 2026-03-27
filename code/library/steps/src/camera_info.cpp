#include "steps/camera_info.hpp"

#include <toml++/toml.h>

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

std::string CameraInfoStep::CacheKey() const {
    std::ostringstream oss;
    oss << sensor_config;

    return caching::CacheKey(cache_key + oss.str());
}

CameraInfo CameraInfoStep::Compute() const {
    auto const result{image_source()};
    if (not result) {
        throw std::runtime_error(
            "we need an error handling strategy for empty image sources to get camera info");  // LCOV_EXCL_LINE
    }
    auto const& [_, img]{*result};

    CameraInfo const camera_info{SensorName(),
                                 ToCameraModel(sensor_config["camera_model"].as_string()->get()),
                                 {0, static_cast<double>(img.size().width), 0, static_cast<double>(img.size().height)}};

    return camera_info;
}

CameraInfo CameraInfoStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    auto const camera_info{database::ReadCameraInfo(db, SensorName())};

    if (not camera_info) {
        throw std::runtime_error("we need a consistent error handling strategy!!!");  // LCOV_EXCL_LINE
    }

    return *camera_info;
}

void CameraInfoStep::Save(CameraInfo const& camera_info,
                          std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(camera_info, db);
}

}  // namespace reprojection::steps
