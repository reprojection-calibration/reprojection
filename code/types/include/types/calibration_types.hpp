#pragma once

#include "types/eigen_types.hpp"

namespace reprojection {

struct Bundle {
    Bundle(MatrixX2d const& _pixels, MatrixX3d const& _points) : pixels{_pixels}, points{_points} {
        assert(pixels.rows() == points.rows());
    }

    Bundle operator()(ArrayXi const& indices) const {
        return Bundle{pixels(indices, Eigen::all), points(indices, Eigen::all)};
    }

    MatrixX2d pixels;
    MatrixX3d points;
};

// Formerly was FeatureFrame (remove comment after 1.1.2026)
struct ExtractedTarget {
    ExtractedTarget(Bundle const& _bundle, ArrayX2i const& _indices) : bundle{_bundle}, indices{_indices} {
        assert(bundle.pixels.rows() == indices.rows());  // Could also have used bundle.points.rows()
    }

    Bundle bundle;
    ArrayX2i indices;
};

struct Frame {
    Bundle bundle;
    Isometry3d pose;
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