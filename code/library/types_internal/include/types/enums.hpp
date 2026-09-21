#pragma once

#include <stdexcept>
#include <string>

// TODO(Jack): Is doxygen really so crazy that I need to add this to the top of the file for this to be generated at
// all?
/**
 * \file enums.hpp
 */

namespace reprojection {

// NOTE(Jack): We turn off code coverage for all the conversions functions because it just does not bring us much here
// If this becomes a problem we can add unit tests for the conversion functions as needed.

// LCOV_EXCL_START

enum class Entity { Camera, Extrinsic, Imu, Target };

inline std::string ToString(Entity const entity_id) {
    if (entity_id == Entity::Camera) {
        return "camera";
    } else if (entity_id == Entity::Extrinsic) {
        return "extrinsic";
    } else if (entity_id == Entity::Imu) {
        return "imu";
    } else if (entity_id == Entity::Target) {
        return "target";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(Entity)");
    }
}

/*! \brief Supported camera projection models.
 *
 *  \par Single focal length
 *  All camera models use a single focal length `f` instead of the standard two focal lengths `fx` and `fy`. Please see
 *  this excellent [article](https://www.tangramvision.com/blog/camera-modeling-focal-length-collinearity) from [Tangram
 *  Vision](https://www.tangramvision.com/) for an explanation.
 *
 *  \par Ucm formulation
 *  We follow the intrinsic parameter conventions from this paper "The Double Sphere Camera Model, Usenko Et al. 2018".
 *  Note that for the "unified camera model" we use the Usenko Et al. proposed numerically stable formulation, and not
 *  the original formulation from "Single view point omnidirectional camera calibration from planar grids, Mei Et al.
 *  2007". The conversion from one to the other is found in Usenko Et al. section 2.2.
 */
enum class CameraModel {
    /// [f, cx, cy, xi, alpha]
    DoubleSphere,
    /// [x, cx, cy, alpha, beta] - "extended unified camera model"
    Eucm,
    /// [f, cx, cy]
    Pinhole,
    /// [f, cx, cy, k1, k2, p1, p2] - "pinhole with four parameter radial-tangential distortion"
    PinholeRadtan4,
    /// [f cx, cy, alpha] - "unified camera model"
    Ucm,
};

inline std::string ToString(CameraModel const camera_model) {
    if (camera_model == CameraModel::DoubleSphere) {
        return "double_sphere";
    } else if (camera_model == CameraModel::Eucm) {
        return "eucm";
    } else if (camera_model == CameraModel::Pinhole) {
        return "pinhole";
    } else if (camera_model == CameraModel::PinholeRadtan4) {
        return "pinhole_radtan4";
    } else if (camera_model == CameraModel::Ucm) {
        return "ucm";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR -Unrecognized argument passed to ToString(CameraModel)");
    }
}

// TODO ADD SHOWINLINESOURCE! UPGRADE DOXYGEN TO DO SO
/*! \brief Convert a human readable snake_case string into a \ref reprojection::CameraModel enum.
 *
 */
inline CameraModel ToCameraModel(std::string_view camera_model) {
    if (camera_model == "double_sphere") {
        return CameraModel::DoubleSphere;
    } else if (camera_model == "eucm") {
        return CameraModel::Eucm;
    } else if (camera_model == "pinhole") {
        return CameraModel::Pinhole;
    } else if (camera_model == "pinhole_radtan4") {
        return CameraModel::PinholeRadtan4;
    } else if (camera_model == "ucm") {
        return CameraModel::Ucm;
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToCameraModel(): " +
                                 std::string(camera_model));
    }
}

// TODO(Jack): Add a description of how to get the numbers of rows and columns and unit dimension for each target type!
/*! \brief Supported calibration targets.
 *
 *  \par Unit Dimension Measurement
 *  To mitigate measurement error measure the board's entire height/width and divide by the corresponding the number of
 *  unit lengths.
 *
 *  \note Reprojection does not support the Kalibr style Aprilgrid, and instead uses the new and improved Aprilgrid3.
 *
 *  \warning Targets must preserve their original dimensions and aspect ratio and must not be stretched, skewed, warped,
 *  or bent.
 */
enum class TargetType {
    /*! \par Unit Dimension
     *  The side length of a single checkerboard square.
     *  \par Size
     *  The number of rows and columns as you would count normally. Note that the internal algorithm uses this value
     *  minus one because the feature extractor actually extracts the internal checkerboard corners.
     */
    Checkerboard,
    /*! \par Unit Dimension
     *  The distance between the center's of two row/column adjacent circles. For asymmetric circle grids this cannot be
     *  measured directly.
     *  \par Size
     *  The total number of rows and columns regardless if symmetric or asymmetric. Please note that the standard OpenCv
     *  asymmetric circle grid specification counts only the asymmetric rows but all the columns to specify the size. Do
     *  not do this!
     */
    CircleGrid,
    /*! \par Unit Dimension
     *  The length of a single square between the corner sharpening elements. If measuring the total internal target
     *  height/width (i.e. excluding the outer sharpening elements) make sure to account for the internal sharpening
     *  elements widths which are each 1/3 `unit_dimension` width.
     *  \par Size
     *  The total number of april tag rows and columns.
     */
    Aprilgrid3,
};

inline std::string ToString(TargetType const target_type) {
    if (target_type == TargetType::Aprilgrid3) {
        return "aprilgrid3";
    } else if (target_type == TargetType::Checkerboard) {
        return "checkerboard";
    } else if (target_type == TargetType::CircleGrid) {
        return "circle_grid";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(TargetType)");
    }
}

// TODO ADD SHOWINLINESOURCE! UPGRADE DOXYGEN TO DO SO
/*! \brief Convert a human readable snake_case string into a \ref reprojection::TargetType enum.
 *
 */
inline TargetType ToTargetType(std::string const& enum_string) {
    if (enum_string == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (enum_string == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (enum_string == "aprilgrid3") {
        return TargetType::Aprilgrid3;
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToTargetType(): " +
                                 enum_string);
    }
}

enum class CacheStatus {
    CacheHit,
    CacheMiss,
};

inline std::string ToString(CacheStatus const status) {
    if (status == CacheStatus::CacheHit) {
        return "cache_hit";
    } else if (status == CacheStatus::CacheMiss) {
        return "cache_miss";
    } else {
        throw std::runtime_error{"LIBRARY IMPLEMENTATION ERROR - ToString(CacheStatus)"};
    }
}
// LCOV_EXCL_STOP

}  // namespace reprojection