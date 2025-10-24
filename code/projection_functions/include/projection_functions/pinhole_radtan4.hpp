#pragma once

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// point_cam is a point in the ideal/normalized/projected camera coordinate frame. What does this actually mean for
// practical purposes? Let us say we have a 3D point that is already in the "camera optical" frame P_co = {x, y, z}. To
// transform this point to the ideal/normalized/projected camera coordinate frame we simply divide P_co by it's z-value
// and then remove the last element.
//
//      Step 1) P_co: {x, y, z} -> {x/z, y/z, 1}
//      Step 2) {x/z, y/z, 1} -> p_cam: {x/z, y/z}
//
// I am making this "ideal perspective projection" more complicated than it needs to be I think, but it does require
// some emphasis here in the context of the radtan4 projection/unprojection. The user and maintainer of aforementioned
// methods need to make sure when using the Radtan4Distortion method that the input p_cam is this
// ideal/normalized/projected camera coordinate, otherwise it will not work properly! Or said another way, we need to
// make sure that we do not accidentally use image coordinates given in pixels here.
template <typename T>
Eigen::Array<T, 2, 1> Radtan4Distortion(Eigen::Array<T, 4, 1> const& distortion, Eigen::Array<T, 2, 1> const& p_cam) {
    T const& x_cam{p_cam[0]};
    T const& y_cam{p_cam[1]};
    T const x_cam2{x_cam * x_cam};
    T const y_cam2{y_cam * y_cam};
    T const r2{x_cam2 + y_cam2};

    T const& k1{distortion[0]};
    T const& k2{distortion[1]};
    T const r_prime{1.0 + (k1 * r2) + (k2 * r2 * r2)};

    T const& p1{distortion[2]};
    T const& p2{distortion[3]};
    T const distorted_x_cam{(r_prime * x_cam) + (2.0 * p1 * x_cam * y_cam) + p2 * (r2 + 2.0 * x_cam2)};
    T const distorted_y_cam{(r_prime * y_cam) + (2.0 * p2 * x_cam * y_cam) + p1 * (r2 + 2.0 * y_cam2)};

    return {distorted_x_cam, distorted_y_cam};
}

// P_co is a 3D point {x, y, z} in the "camera optical" frame expressed
template <typename T>
Eigen::Array<T, 2, 1> PinholeRadtan4Projection(Eigen::Array<T, 8, 1> const& intrinsics,
                                               Eigen::Array<T, 3, 1> const& P_co) {
    T const& x{P_co[0]};
    T const& y{P_co[1]};
    T const& z{P_co[2]};
    T const x_cam{x / z};
    T const y_cam{y / z};
    Eigen::Array<T, 2, 1> const p_cam{x_cam, y_cam};

    Eigen::Array<T, 2, 1> const distorted_p_cam{Radtan4Distortion<T>(intrinsics.bottomRows(4), p_cam)};
    Eigen::Array<T, 3, 1> const P_star{distorted_p_cam[0], distorted_p_cam[1], 1};

    // NOTE(Jack): Because we already did the ideal projective transform to the camera coordinate frame above
    // (i.e. when we built p_cam), the pinhole projection here is actually only being used to convert the distorted
    // point P_star into image pixel coordinates. In this sense P_star is itself a confusing construct because it
    // masquerades as a 3D point but intuitively it does not have nearly the amount of "freedom" at this point when
    // compared to the input P_co which was a real 3D point. That is the reason that I do not use a frame postfix like
    // "_co" and instead just call it "_star".
    return PinholeProjection<T>(intrinsics.topRows(4), P_star);
}

// NOTE(Jack): Here we are using ceres to calculate the jacobian of the Radtan4Distortion. Unlike most ceres "functors"
// you will see, this is NOT a "cost" functor! It will not return the residual between some predicted and measured value
// or anything like that. Instead it simply calls Radtan4Distortion and returns the value of the distorted point in the
// place where the residual would normally be returned via the pointer.
//
// The jacobian calculated here is exactly the same (maybe there is a sign flip depending on how you do it) as if we
// would have provided the measured value and calculated a residual instead. I guess in the context of calculating a
// jacobian the "measured" value is a constant and therefore does actually effect the jacobian :) It might look like a
// ceres cost functor because we need to fit into how ceres does it, and therefore at first glance you might be
// confused what is going on here.
struct Radtan4DistortionFunctor {
    Radtan4DistortionFunctor(Eigen::Array<double, 4, 1> const& distortion) : distortion_{distortion} {}

    // WARN(Jack): This is not a cost function operator! Read the context above in the note.
    template <typename T>
    bool operator()(T const* const p_cam_ptr, T* const distorted_p_cam_ptr) const {
        Eigen::Map<Eigen::Array<T, 2, 1> const> const p_cam(p_cam_ptr);
        Eigen::Array<T, 2, 1> const distorted_p_cam{Radtan4Distortion<T>(distortion_.cast<T>(), p_cam)};

        distorted_p_cam_ptr[0] = distorted_p_cam[0];
        distorted_p_cam_ptr[1] = distorted_p_cam[1];

        return true;
    }

   private:
    Eigen::Array<double, 4, 1> distortion_;
};

std::tuple<Eigen::Array2d, Eigen::Matrix2d> Radtan4DistortionJacobianUpdate(Eigen::Array4d const& distortion,
                                                                            Eigen::Array2d const& p_cam) {
    // NOTE(Jack): The ceres type is AutoDiffCostFunction, but as we wrote above, this is not actually calculating a
    // residual cost! See above.
    // TODO(Jack): Do we need to manually deallocate this?
    auto* const cost_function{
        new ceres::AutoDiffCostFunction<Radtan4DistortionFunctor, 2, 2>(new Radtan4DistortionFunctor(distortion))};

    // This is a super annoying way to initialize the data for the format required by the Evaluate function, nothing
    // else I can do... Here and below.
    double const p_cam_array[2]{p_cam[0], p_cam[1]};
    double const* p_cam_ptr{p_cam_array};
    double const* const* p_cam_ptr_ptr{&p_cam_ptr};

    double distorted_p_cam_ptr[2];
    double J_ptr[2 * 2];
    double* J_ptr_ptr[4]{J_ptr};

    // TODO(Jack): What would we do if the evaluation here was not successful? Is that even a worry we need to consider?
    // Right now we add an assertion so we can catch failures in development.
    bool success{cost_function->Evaluate(p_cam_ptr_ptr, distorted_p_cam_ptr, J_ptr_ptr)};
    assert(success);

    Eigen::Vector2d const distorted_p_cam{distorted_p_cam_ptr[0], distorted_p_cam_ptr[1]};
    Eigen::Matrix2d const J{Eigen::Map<const Eigen::Matrix2d>(J_ptr)};

    return {distorted_p_cam, J};
}

// TODO(Jack): We are manually do an optimization here, therefore I am not sure if we can get jacobians here like a
// classic templated "pass through" ceres autodiff capable function. Therefore maybe it does not make sense to template
// here at all.
template <typename T>
Eigen::Vector<T, 3> PinholeRadtan4Unprojection(Eigen::Array<T, 8, 1> const& intrinsics,
                                               Eigen::Array<T, 2, 1> const& pixel) {
    Eigen::Array<T, 4, 1> const pinhole_intrinsics{intrinsics.topRows(4)};
    Eigen::Array<T, 3, 1> const ray{PinholeUnprojection(pinhole_intrinsics, pixel)};  // Normalized image plane ray

    // TODO(Jack): How many iterations do we really need here?
    Eigen::Array<T, 4, 1> const radtan4_distortion{intrinsics.bottomRows(4)};
    Eigen::Vector2d const y{ray.topRows(2)};
    Eigen::Vector2d ybar{y};
    Eigen::Vector2d y_tmp;
    for (int i{0}; i < 5; ++i) {
        y_tmp = ybar;
        auto const [xxx, J]{Radtan4DistortionJacobianUpdate(radtan4_distortion, y_tmp)};
        y_tmp = xxx;

        Eigen::Vector2d const e{y - y_tmp};
        Eigen::Vector2d const du{(J.transpose() * J).inverse() * J.transpose() * e};
        ybar += du;

        // TODO(Jack): Check error and exit early if the error is small
    }

    return {ybar[0], ybar[1], 1.0};
}

}  // namespace reprojection::projection_functions