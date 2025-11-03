#pragma once

#include "types/eigen_types.hpp"

namespace reprojection {

struct FeatureFrame {
    MatrixX2d pixels;
    MatrixX3d points;
    ArrayX2i indices;
};

// NOTE(Jack): We need this idea at some point, because somewhere along the line the user is gonna have a config file or
// a button that says which camera they want to calibrate. Where this actually ends up, or where the final battle line
// between templated and non templated code is, we will find out later maybe. But for now it stands here.
// ERROR(Jack): The intrinsic size information is currently coded in two separate places! Once here in the enum and one
// again in each projection class! This will be a pain point going forward if we do not find a solution here to
// eliminate the manual and far apart duplication.
enum class CameraModel {
    DoubleSphere = 6,  //
    Pinhole = 4,
    PinholeRadtan4 = 8,
    UnifiedCameraModel = 5
};

}  // namespace reprojection