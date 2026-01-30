#pragma once

#include <stdexcept>
#include <string>

namespace reprojection {

// ERROR(Jack): The intrinsic size information is currently coded in two separate places! Once here in the enum and one
// again in each projection class! This will be a pain point going forward if we do not find a solution here to
// eliminate the manual and far apart duplication.
enum class CameraModel {
    DoubleSphere = 6,  //
    Pinhole = 4,
    PinholeRadtan4 = 8,
    UnifiedCameraModel = 5,
};

enum class TargetType {
    Checkerboard,
    CircleGrid,
    AprilGrid3,
};

// TODO(Jack): Is this the right place to put functions like this? What about testing?
inline TargetType ToTargetType(std::string const& enum_string) {
    if (enum_string == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (enum_string == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (enum_string == "april_grid3") {
        return TargetType::AprilGrid3;
    } else {
        throw std::runtime_error("Unrecognized argument passed to ToTargetType(): " + enum_string);  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection