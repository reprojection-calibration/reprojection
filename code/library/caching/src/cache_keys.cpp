#include "caching/cache_keys.hpp"

#include "cache_key_from.hpp"

namespace reprojection::caching {

std::string CacheKey(std::string_view const& data) { return CacheKeyFrom(data); }

std::string CacheKey(std::string_view config, EncodedImages const& encoded_images) {
    return CacheKeyFrom(config, encoded_images);
}

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

}  // namespace reprojection::caching
