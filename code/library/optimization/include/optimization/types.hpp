#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"

namespace reprojection::optimization {

struct DiscreteRig {
    AssetId asset_id;
    Frames frames;
};

struct ContinuousRig {
    AssetId asset_id;
    spline::Se3Spline spline;
};

}  // namespace reprojection::optimization