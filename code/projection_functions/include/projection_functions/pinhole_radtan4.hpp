#pragma once

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

/**
 * \ingroup projection_classes
 * \brief Classic four parameter radial-tangential distortion model on top of a pinhole camera.
 */
struct PinholeRadtan4 {
    static int constexpr Size{8};

    /**
     * \brief Applies four parameter radtan distortion to a 2D point in the ideal/normalized camera frame, producing a
     * "distorted" point in that same frame.
     */
    template <typename T>
    static Eigen::Array<T, 2, 1> Distort(Eigen::Array<T, 4, 1> const& distortion, Eigen::Array<T, 2, 1> const& p_cam) {
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

    template <typename T>
    static Eigen::Array<T, 2, 1> Project(Eigen::Array<T, Size, 1> const& intrinsics,
                                         Eigen::Array<T, 3, 1> const& P_co) {
        T const& x{P_co[0]};
        T const& y{P_co[1]};
        T const& z{P_co[2]};
        T const x_cam{x / z};
        T const y_cam{y / z};
        Eigen::Array<T, 2, 1> const p_cam{x_cam, y_cam};

        Eigen::Array<T, 2, 1> const distorted_p_cam{Distort<T>(intrinsics.bottomRows(4), p_cam)};
        Eigen::Array<T, 3, 1> const P_star{distorted_p_cam[0], distorted_p_cam[1], T(1)};  // Set z=1

        // NOTE(Jack): Because we already did the ideal projective transform to the camera coordinate frame above when
        // we built p_cam, the pinhole projection below is actually only being used to convert the distorted point
        // P_star into image pixel coordinates. In this sense P_star is itself a confusing construct because it
        // masquerades as a 3D point, but it does not have nearly the amount of "freedom" at this point when compared to
        // the input P_co which was a real 3D point. It is constrained to the ideal/normalized camera frame. That is the
        // reason that I do not use a frame postfix like "_co" and instead just call it "_star".
        return Pinhole::Project<T>(intrinsics.topRows(4), P_star);
    }

    static Array3d Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Array2d const& pixel);

    static std::tuple<Array2d, Eigen::Matrix2d> JacobianUpdate(Array4d const& distortion, Array2d const& p_cam);

    // NOTE(Jack): Here we are using ceres to calculate the jacobian of the PinholeRadtan4::Distort. Unlike most ceres
    // "functors" you will see, this is NOT a "cost" functor! It will not return the residual between some predicted and
    // measured value or anything like that. Instead it simply calls PinholeRadtan4::Distort and returns the value of
    // the distorted point in the place where the residual would normally be returned via the pointer.
    //
    // The jacobian calculated here is exactly the same (maybe there is a sign flip depending on how you do it) as if we
    // would have provided the measured value and calculated a residual instead. I guess in the context of calculating a
    // jacobian the "measured" value is a constant and therefore does actually affect the jacobian :) It might look like
    // a ceres cost functor because we need to fit into how ceres does it, and therefore at first glance you might be
    // confused what is going on here.
    struct DistortFunctor {
        explicit DistortFunctor(Eigen::Array<double, 4, 1> const& distortion) : distortion_{distortion} {}

        // WARN(Jack): This is not a cost function operator! Read the context above in the note.
        template <typename T>
        bool operator()(T const* const p_cam_ptr, T* const distorted_p_cam_ptr) const {
            Eigen::Map<Eigen::Array<T, 2, 1> const> const p_cam(p_cam_ptr);
            Eigen::Array<T, 2, 1> const distorted_p_cam{Distort<T>(distortion_.cast<T>(), p_cam)};

            distorted_p_cam_ptr[0] = distorted_p_cam[0];
            distorted_p_cam_ptr[1] = distorted_p_cam[1];

            return true;
        }

       private:
        Eigen::Array<double, 4, 1> distortion_;
    };
};

}  // namespace reprojection::projection_functions