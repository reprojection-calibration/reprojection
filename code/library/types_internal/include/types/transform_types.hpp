#pragma once

#include <map>

#include "types/database_types.hpp"
#include "types/eigen_types.hpp"

#include "calibration_types.hpp"

namespace reprojection {

struct Extrinsic {
    AssetId frame_a;
    AssetId frame_b;
    Array6d se3_a_b;
};

}  // namespace reprojection