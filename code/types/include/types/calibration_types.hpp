#pragma once

#include "types/eigen_types.hpp"

namespace reprojection {

struct Bundle {
    MatrixX2d pixels;
    MatrixX3d points;
};

struct ExtractedTarget {
    Bundle bundle;
    ArrayX2i indices;
};

// TODO NAME!!!
struct Frame {
    Bundle bundle;
    Isometry3d pose;
};

// Output of the feature extractor
struct FeatureFrame {
    MatrixX2d pixels;
    MatrixX3d points;
    ArrayX2i indices;
};

// Output of the multiple view geometry data generator
struct MvgFrame {
    Isometry3d pose;
    MatrixX2d pixels;
    MatrixX3d points;
};

// TODO(Jack): There is some unclear consistency here! Right now we are coding the testing data generator mock
// terminology into a very generic project wide data type!
// TODO(Jack): Should we just stop beating around the bush and make the mvg frame generator also return the ids? By
// pretending that it is not generating feature frames we make it more complicate than it needs to be, even though we
// only have the 3d part so we can test the dlt algo that I accidentally implemented in the first place.

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