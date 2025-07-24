#pragma once

#include <ceres/rotation.h>

#include <array>

namespace reprojection_calibration::reprojection {

// NOTE(Jack): It might be that this function and others like it belong in an "optimization" or ceres namespace. All
// functions that take raw pointers as input are that way only because I want to use them with the optimization
// framework. In general with respect to this function we will probably need to define another one that does not depend
// on ceres for use in the rest of the code base because we do not want to add a ceres depdendency everywhere.
template <typename T>
std::array<T, 3> TransformPoint(T const* const tf, T const* const point) {
    T const& e_x{tf[0]};
    T const& e_y{tf[1]};
    T const& e_z{tf[2]};
    T const axis_angle[3]{e_x, e_y, e_z};

    T transformed_point[3];
    ceres::AngleAxisRotatePoint(axis_angle, point, transformed_point);

    T const& x{tf[3]};
    T const& y{tf[4]};
    T const& z{tf[5]};

    transformed_point[0] = transformed_point[0] + x;
    transformed_point[1] = transformed_point[1] + y;
    transformed_point[2] = transformed_point[2] + z;

    return {transformed_point[0], transformed_point[1], transformed_point[2]};
}

}  // namespace reprojection_calibration::reprojection