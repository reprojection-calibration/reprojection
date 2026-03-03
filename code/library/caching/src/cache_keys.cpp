#include "caching/cache_keys.hpp"

#include "hashing.hpp"
#include "serialize.hpp"

namespace reprojection::caching {

std::string CacheKey(CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& intrinsics) {
    std::string const data{Serialize(sensor) + Serialize(targets) + Serialize(intrinsics)};

    return Sha256(data);
}

}  // namespace reprojection::caching
