#pragma once

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

enum class TomlParseError {
    IncorrectType,
    MissingKey,
    UnknownKey,
};

enum class TomlType {
    Array,
    Boolean,
    FloatingPoint,
    Integer,
    String,
    Table,
};

}  // namespace reprojection