#pragma once

#include <memory>

#include "projection_functions/camera_model.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct MvgHelpers {
    static std::tuple<MatrixX2d, ArrayXb> Project(MatrixX3d const& points_w,
                                                  std::unique_ptr<projection_functions::Camera> const& camera,
                                                  Isometry3d const& tf_co_w);

    // TODO(Jack): Refactor the class to explicitly generate targets, there is no reason to pretend it also needs to do
    //  random points.
    static MatrixX3d BuildTargetPoints(bool const flat);
};

}  // namespace reprojection::testing_mocks