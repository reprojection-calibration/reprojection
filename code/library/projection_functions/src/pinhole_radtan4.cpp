#include "projection_functions/pinhole_radtan4.hpp"

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

Array3d PinholeRadtan4::Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Array2d const& pixel) {
    Array3d const P_ray{Pinhole::Unproject(intrinsics.head<4>(), pixel)};
    Vector2d const p_cam_0{P_ray.head<2>()};

    // TODO(Jack): How many iterations do we really need here?
    // TODO(Jack): Check error and exit early if the error is small, will this save us time?
    // NOTE(Jack): The name part "*_n" signifies that this this is where we accumulate the result and have the final
    // answer after n iterations. Inside the loop we use the name part "_i" to demonstrate that it is a variable that
    // will only exist for the i'th iteration and be overwritten next time.
    Vector2d distorted_p_cam_n{p_cam_0};
    for (int i{0}; i < 5; ++i) {
        auto const [distorted_p_cam_i, J]{JacobianUpdate(intrinsics.tail<4>(), distorted_p_cam_n)};

        Vector2d const e{distorted_p_cam_i.matrix() - p_cam_0};
        Vector2d const du{(J.transpose() * J).inverse() * J.transpose() * e};
        distorted_p_cam_n -= du;
    }

    return {distorted_p_cam_n[0], distorted_p_cam_n[1], 1.0};
}

std::tuple<Array2d, Matrix2d> PinholeRadtan4::JacobianUpdate(Array4d const& distortion, Array2d const& p_cam) {
    // TODO(Jack): Do we need to manually deallocate this?
    auto* const function{new ceres::AutoDiffCostFunction<DistortFunctor, 2, 2>(new DistortFunctor(distortion))};

    // This is a super annoying way to initialize the data for the format required by the Evaluate function, nothing
    // else I can do... Here and below for the jacobian.
    double const p_cam_array[2]{p_cam[0], p_cam[1]};
    double const* p_cam_ptr{p_cam_array};
    double const* const* p_cam_ptr_ptr{&p_cam_ptr};

    double distorted_p_cam_ptr[2];
    double J_ptr[2 * 2];
    double* J_ptr_ptr[4]{J_ptr};

    // TODO(Jack): What would we do if the evaluation here was not successful? Is that even a worry we need to consider?
    // Right now we add an assertion so we can catch failures in debug builds.
    bool success{function->Evaluate(p_cam_ptr_ptr, distorted_p_cam_ptr, J_ptr_ptr)};
    assert(success);
    static_cast<void>(success);

    Vector2d const distorted_p_cam{distorted_p_cam_ptr[0], distorted_p_cam_ptr[1]};
    Matrix2d const J{Eigen::Map<const Matrix2d>(J_ptr)};

    return {distorted_p_cam, J};
}

}  // namespace reprojection::projection_functions