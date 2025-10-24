#pragma once

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Consistent naming of the point/pixel!
template <typename T>
Eigen::Vector<T, 2> Radtan4Distortion(Eigen::Array<T, 4, 1> const& distortion,
                                      Eigen::Array<T, 2, 1> const& projected_point) {
    T const& x_z{projected_point[0]};
    T const& y_z{projected_point[1]};
    T const x_z2{x_z * x_z};
    T const y_z2{y_z * y_z};
    T const r2{x_z2 + y_z2};

    T const& k1{distortion[0]};
    T const& k2{distortion[1]};
    T const r_prime{1.0 + (k1 * r2) + (k2 * r2 * r2)};

    T const& p1{distortion[2]};
    T const& p2{distortion[3]};
    T const x_star{(r_prime * x_z) + (2.0 * p1 * x_z * y_z) + p2 * (r2 + 2.0 * x_z2)};
    T const y_star{(r_prime * y_z) + (2.0 * p2 * x_z * y_z) + p1 * (r2 + 2.0 * y_z2)};

    return {x_star, y_star};
}

template <typename T>
Eigen::Vector<T, 2> PinholeRadtan4Projection(Eigen::Array<T, 8, 1> const& intrinsics,
                                             Eigen::Array<T, 3, 1> const& point) {
    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const x_z{x / z};
    T const y_z{y / z};
    Eigen::Array<T, 2, 1> const projected_point{x_z, y_z};

    Eigen::Array<T, 4, 1> const distortion{intrinsics.bottomRows(4)};
    Eigen::Array<T, 2, 1> const projected_point_distorted{Radtan4Distortion(distortion, projected_point)};

    Eigen::Array<T, 4, 1> const pinhole_intrinsics{intrinsics.topRows(4)};
    Eigen::Array<T, 3, 1> const point_star{projected_point_distorted[0], projected_point_distorted[1],
                                           1};  // z=1 here because we already "projected" at the start

    return PinholeProjection<T>(pinhole_intrinsics, point_star);
}

// UNDISTORTION BELOW

// TODO(Jack): Make this just return the distorted value also!
// TODO(Jack): Would it be possible to repurpose the residual to return the point directly? Or do we need the residual
// to calculate the derivative?
struct Radtan4DistortionCostFunctor {
    Radtan4DistortionCostFunctor(Eigen::Array<double, 4, 1> const& distortion) : distortion_{distortion} {}

    // TODO(Jack): Clarify what is a point and what is a pixel! Unit/plane/normalized???
    // TODO(Jack): Inconsistent use of array and vector
    // Is this really a pure pixel of an image plane constrained 3D projected point?
    template <typename T>
    bool operator()(T const* const pixel_ptr, T* const residual) const {
        Eigen::Map<Eigen::Array<T, 2, 1> const> const pixel(pixel_ptr);
        Eigen::Vector<T, 2> const pixel_star{projection_functions::Radtan4Distortion<T>(distortion_.cast<T>(), pixel)};

        residual[0] = pixel_star[0];
        residual[1] = pixel_star[1];

        return true;
    }

   private:
    Eigen::Array<double, 4, 1> distortion_;
};

// TODO(Jack): This should take a 2d pixel not a 3d point! Figure out some consistency here!
// TODO(Jack): Must be called with a normalized image plane point/pixel! z=1
// TODO(Jack): Rename to reflect the fact that we ONLY get the derivative, forget the residual error! Do not even need
// to return the residual because it alwazs should be zero right?
std::tuple<Vector2d, Eigen::Matrix2d> Radtan4DistortionUpdate(Eigen::Array<double, 4, 1> const& radtan4_distortion,
                                                              Eigen::Array2d const& image_plane_point) {
    auto* cost_function = new ceres::AutoDiffCostFunction<Radtan4DistortionCostFunctor, 2, 2>(
        new Radtan4DistortionCostFunctor(radtan4_distortion));

    const double values[2] = {image_plane_point[0], image_plane_point[1]};
    const double* pValues = values;
    const double* const* pixel = &pValues;

    double residuals[2];
    double jacobians_data[2 * 2];
    double* jacobians[] = {jacobians_data};

    // TODO(Jack): What would we do if the evaluation here was not successful?
    bool success{cost_function->Evaluate(pixel, residuals, jacobians)};
    static_cast<void>(success);

    Eigen::Vector2d const e{residuals[0], residuals[1]};
    Eigen::Matrix2d const J{Eigen::Map<const Eigen::Matrix2d>(jacobians_data)};

    return {e, J};
}

// TODO(Jack): We are manually do an optimization here, therefore I am not sure if we can get jacobians here like a
// classic templated "pass through" ceres autodiff capable function. Therefore maybe it does not make sense to template
// here at all.
template <typename T>
Eigen::Vector<T, 3> PinholeRadtan4Unprojection(Eigen::Array<T, 8, 1> const& intrinsics,
                                               Eigen::Array<T, 2, 1> const& pixel) {
    Eigen::Array<T, 4, 1> const pinhole_intrinsics{intrinsics.topRows(4)};
    Eigen::Array<T, 3, 1> const ray{PinholeUnrojection(pinhole_intrinsics, pixel)};  // Normalized image plane ray

    // TODO(Jack): How many iterations do we really need here?
    Eigen::Array<T, 4, 1> const radtan4_distortion{intrinsics.bottomRows(4)};
    Eigen::Vector2d const y{ray.topRows(2)};
    Eigen::Vector2d ybar{y};
    Eigen::Vector2d y_tmp;
    for (int i{0}; i < 5; ++i) {
        y_tmp = ybar;
        auto const [_, J]{Radtan4DistortionUpdate(radtan4_distortion, y_tmp)};
        y_tmp = Radtan4Distortion<T>(radtan4_distortion, y_tmp);

        Eigen::Vector2d const e{y - y_tmp};
        Eigen::Vector2d const du{(J.transpose() * J).inverse() * J.transpose() * e};
        ybar += du;

        // TODO(Jack): Check error and exit early if the error is small
    }

    return {ybar[0], ybar[1], 1.0};
}

}  // namespace reprojection::projection_functions