#include "projection_cost_function.hpp"

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// WARN(Jack): This function will default to the UCM model if a match is not found! Some people might expect
// it to actually fail if the requested model is not present! Rethink this if this turns into a problem! I.e. throw and
// error and kill the program because it indicates that this function was not updated as new camera models were added.
ceres::CostFunction* Create(CameraModel const projection_type, Vector2d const& pixel, Vector3d const& point) {
    if (projection_type == CameraModel::DoubleSphere) {
        return Create_T<projection_functions::DoubleSphere>(pixel, point);
    } else if (projection_type == CameraModel::Pinhole) {
        return Create_T<projection_functions::Pinhole>(pixel, point);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return Create_T<projection_functions::PinholeRadtan4>(pixel, point);
    } else {
        return Create_T<projection_functions::UnifiedCameraModel>(pixel, point);
    }
}

}  // namespace  reprojection::optimization
