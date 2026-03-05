#include "caching/cache_keys.hpp"

#include "hashing.hpp"
#include "serialize.hpp"

namespace reprojection::caching {

// TODO(Jack): Can we replace these methods here with some "meta programming" or something like that? It is going to get
//  very repetitive here.
std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements) {
    std::string const data{Serialize(camera_info) + Serialize(camera_measurements)};

    return Sha256(data);
}

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     CameraState const& camera_state) {
    std::string const data{Serialize(camera_info) + Serialize(camera_measurements) + Serialize(camera_state)};

    return Sha256(data);
}

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     OptimizationState const& optimization_state) {
    std::string const data{Serialize(camera_info) + Serialize(camera_measurements) +
                           Serialize(optimization_state.camera_state) + Serialize(optimization_state.frames)};

    return Sha256(data);
}

}  // namespace reprojection::caching
