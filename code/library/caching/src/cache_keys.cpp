#include "caching/cache_keys.hpp"

#include "cache_key_from.hpp"

namespace reprojection::caching {

// TODO(Jack): Can we replace these methods here with some "meta programming" or something like that? It is going to get
//  very repetitive here.
std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements) {
    return CacheKeyFrom(camera_info, camera_measurements);
}

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     CameraState const& camera_state) {
    return CacheKeyFrom(camera_info, camera_measurements, camera_state);
}

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     OptimizationState const& optimization_state) {
    return CacheKeyFrom(camera_info, camera_measurements, optimization_state.camera_state, optimization_state.frames);
}

std::string CacheKey(TargetInfo const& target_info, EncodedImages const& encoded_images, std::string_view config) {
    return CacheKeyFrom(target_info, encoded_images, config);
}

std::string CacheKey(std::string_view const& data) { return CacheKeyFrom(data); }

std::string CacheKey(std::string_view sensor_name, CameraModel const camera_model,
                     EncodedImages const& encoded_images) {
    return CacheKeyFrom(sensor_name, camera_model, encoded_images);
}

std::string CacheKey(std::string_view sensor_name, ImuMeasurements const& imu_data,
                     Eigen::Matrix<double, 6, -1> const& control_points, uint64_t const t0_ns,
                     uint64_t const delta_t_ns) {
    return CacheKeyFrom(sensor_name, imu_data, control_points, std::to_string(t0_ns), std::to_string(delta_t_ns));
}

std::string CacheKey(std::string_view sensor_name, Frames const& frames) { return CacheKeyFrom(sensor_name, frames); }

}  // namespace reprojection::caching
