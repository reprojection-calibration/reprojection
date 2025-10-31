#pragma once

#include "ceres_geometry.hpp"
#include "projection_functions/projection_class_concept.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): Where should this actually live?
enum class CameraModel {
    DoubleSphere,  //
    Pinhole,
    PinholeRadtan4,
    UnifiedCameraModel
};

ceres::CostFunction* Create(CameraModel const projection_type, Vector2d const& pixel, Vector3d const& point);

// NOTE(Jack): Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
// TODO(Jack): Do we need to template the entire class? Or can we do just the subfunctions?
// WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
// referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
// that but consider how we  can design the program to handle that automatically.
// NOTE(Jack): The operator() method  is the contact line were ceres requirements for using raw pointers hits
// our desire to use more expressive eigen types. That is why here in the operator() we find the usage of the
// Eigen::Map class.
template <typename T_Model>
    requires projection_functions::ProjectionClass<T_Model>
class ProjectionCostFunction_T {
   public:
    explicit ProjectionCostFunction_T(Vector2d const& pixel, Vector3d const& point) : pixel_{pixel}, point_{point} {}

    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const pose_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 6> const> pose(pose_ptr);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Map<Eigen::Array<T, T_Model::Size, 1> const> intrinsics(intrinsics_ptr);
        Eigen::Vector<T, 2> const pixel{T_Model::template Project<T>(intrinsics, point_co)};

        residual[0] = T(pixel_[0]) - pixel[0];
        residual[1] = T(pixel_[1]) - pixel[1];

        return true;
    }

    Vector2d pixel_;
    Vector3d point_;
};

// TODO(Jack): In ceres examples this create method is normally a static member of the cost function itself. However
// here because of how we are using templates, my assessment at this time is that this is not possible, and also
// not necessary.
template <typename T_Model>
    requires projection_functions::ProjectionClass<T_Model>
ceres::CostFunction* Create_T(Vector2d const& pixel, Vector3d const& point) {
    using ProjectionCostFunction = ProjectionCostFunction_T<T_Model>;

    return new ceres::AutoDiffCostFunction<ProjectionCostFunction, 2, T_Model::Size, 6>(
        new ProjectionCostFunction(pixel, point));
}

}  // namespace  reprojection::optimization
