#include "projection_functions/pinhole_radtan4.hpp"

#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::projection_functions;

Eigen::Array<double, 8, 1> const pinhole_radtan4_intrinsics{600, 600, 360, 240, -0.1, 0.1, 0.001, 0.001};
Eigen::MatrixX3d const gt_points{{0, 0, 10}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};
// Heuristic groundtruth values caculated by running the projection functions itself once - hacky!
Eigen::MatrixX2d const gt_pixels{{pinhole_radtan4_intrinsics[2], pinhole_radtan4_intrinsics[3]},
                                 {8.9424000000000206, 240.21600000000001},
                                 {712.35359999999991, 240.21600000000001},
                                 {360.096, 3.5135999999999683},
                                 {360.096, 477.06240000000003}};

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeRadtan4Projection) {
    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(PinholeRadtan4Projection<double>(pinhole_radtan4_intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeEquivalentProjection) {
    // If [k1, k2, p1, p2] are zero then pinhole radtan4 should essentially just act as a pinhole camera.
    Eigen::Array<double, 8, 1> const pinhole_intrinsics{600, 600, 360, 240, 0, 0, 0, 0};
    Eigen::MatrixX2d const gt_pinhole_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                                             {0, pinhole_intrinsics[3]},
                                             {720, pinhole_intrinsics[3]},
                                             {pinhole_intrinsics[2], 0},
                                             {pinhole_intrinsics[2], 480}};

    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(PinholeRadtan4Projection<double>(pinhole_intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pinhole_pixels.row(i).transpose()));
    }
}

struct Radtan4DistortionCostFunctor {
    Radtan4DistortionCostFunctor(Eigen::Array<double, 4, 1> const& distortion, Eigen::Array2d const& pixel)
        : distortion_{distortion}, pixel_{pixel} {}

    // TODO(Jack): Clarify what is a point and what is a pixel! Unit/plane/normalized???
    // TODO(Jack): Inconsistent use of array and vector
    // Is this really a pure pixel of an image plane constrained 3D projected point?
    template <typename T>
    bool operator()(T const* const pixel_ptr, T* const residual) const {
        Eigen::Map<Eigen::Array<T, 2, 1> const> const pixel(pixel_ptr);
        Eigen::Vector<T, 2> const pixel_star{projection_functions::Radtan4Distortion<T>(distortion_.cast<T>(), pixel)};

        residual[0] = T(pixel_[0]) - pixel_star[0];
        residual[1] = T(pixel_[1]) - pixel_star[1];

        return true;
    }

   private:
    Eigen::Array<double, 4, 1> distortion_;
    Vector2d pixel_;
};

// TODO(Jack): This should take a 2d pixel not a 3d point! Figure out some consistency here!
// TODO(Jack): Must be called with a normalized image plane point/pixel! z=1
std::tuple<Vector2d, Eigen::Matrix2d> Xxx(Eigen::Array<double, 4, 1> const& radtan4_distortion,
                                          Eigen::Array2d const& image_plane_point) {
    auto* cost_function = new ceres::AutoDiffCostFunction<Radtan4DistortionCostFunctor, 2, 2>(
        new Radtan4DistortionCostFunctor(radtan4_distortion, image_plane_point));

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

TEST(ProjectionFunctionsPinholeRadtan4, TestRadtan4DistortionCostFunctor) {
    Eigen::Array<double, 4, 1> const radtan4_distortion{-0.1, 0.1, 0.001, 0.001};
    // What kind of guarantee/requirement do we have that the z value of the ray is z=1?
    Eigen::Array2d const image_plane_point{-0.1, -0.1};
    auto const [e, J]{Xxx(radtan4_distortion, image_plane_point)};

    EXPECT_FLOAT_EQ(e[0], -0.00025600000000000622);
    EXPECT_FLOAT_EQ(e[1], -0.00025600000000000622);
    EXPECT_FLOAT_EQ(J(0, 0), -0.99531999999999998);
    EXPECT_FLOAT_EQ(J(0, 1), 0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 0), 0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 1), -0.99531999999999998);
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
    Eigen::Vector2d undistorted_ray{ray.topRows(2)};
    for (int i{0}; i < 5; ++i) {
        auto const [e, J]{Xxx(radtan4_distortion, undistorted_ray)};

        Eigen::Vector2d const du{(J.transpose() * J).inverse() * J.transpose() * e};
        undistorted_ray += du;
    }

    return {undistorted_ray[0], undistorted_ray[1], 1.0};
}

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeRadtan4Unprojection) {
    Eigen::Vector3d const ray{PinholeRadtan4Unprojection<double>(pinhole_radtan4_intrinsics, {370, 250})};
    EXPECT_TRUE(ray.isApprox(Eigen::Vector3d{0.016670373066475275, 0.016670373066475275, 1}));
}