#include "caching/cache_keys.hpp"

#include "hashing.hpp"
#include "serialize.hpp"

namespace reprojection::caching {

// TODO(Jack): Ca we replace these methods here with some "meta programming" or something like that? It is going to get
//  very repetitive here.
std::string CacheKey(CameraInfo const& sensor, CameraMeasurements const& targets) {
    std::string const data{Serialize(sensor) + Serialize(targets)};

    return Sha256(data);
}

std::string CacheKey(CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& intrinsics) {
    std::string const data{Serialize(sensor) + Serialize(targets) + Serialize(intrinsics)};

    return Sha256(data);
}

}  // namespace reprojection::caching
