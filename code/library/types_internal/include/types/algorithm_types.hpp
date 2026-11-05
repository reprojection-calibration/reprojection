#pragma once

#include <vector>

#include <feature_extraction/types.hpp>

#include "types/eigen_types.hpp"

namespace reprojection {

// NOTE(Jack): Both of these types (Bundle and ExtractedTarget) used to be fully defined here. But when I refactored the
// feature extraction code into its own package I needed move them too. Here we use a `using` alias to bring the
// external namespace in as if the two types were local types. This helped prevent us from having to make any code
// changes even though we are now using the types from the external feature extraction package.
using Bundle = feature_extraction::Bundle;
using ExtractedTarget = feature_extraction::ExtractedTarget;

struct ImuData {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

struct ImageBuffer {
    std::vector<unsigned char> data;
};

}  // namespace reprojection