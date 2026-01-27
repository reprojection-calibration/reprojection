#include "projection_functions/double_sphere.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "projection_functions/double_sphere.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Array6d const double_sphere_intrinsics{600, 600, 360, 240, 0.1, 0.2};
MatrixX2d const gt_pixels{{double_sphere_intrinsics[2], double_sphere_intrinsics[3]},
                          {46.087794035716172, double_sphere_intrinsics[3]},
                          {673.83161575882514, double_sphere_intrinsics[3]},
                          {double_sphere_intrinsics[2], 26.040025446950807},
                          {double_sphere_intrinsics[2], 453.87414877978961}};

TEST(ProjectionFunctionsDoubleSphere, TestDoubleSphereProject) {
    auto const camera{
        projection_functions::DoubleSphereCamera(double_sphere_intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask](camera.Project(testing_utilities::gt_points));
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsDoubleSphere, TestPinholeEquivalentProjection) {
    // If xi and alpha are zero then double sphere should essentially just act as a pinhole camera.
    Array6d const pinhole_intrinsics{600, 600, 360, 240, 0, 0};
    MatrixX2d const pinhole_pixels{{360, 240},  //
                                   {0, 240},
                                   {719.9, 240},
                                   {360, 0},
                                   {360, 479.9}};

    auto const camera{projection_functions::DoubleSphereCamera(pinhole_intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask]{camera.Project(testing_utilities::gt_points)};
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(pinhole_pixels));
}

// TODO(Jack): This currently only tests the underlying pinhole projection masking (i.e. behind the camera is invalid).
// Add the real double sphere validity checks from the paper!
TEST(ProjectionFunctionsPinhole, TestDoubleSphereProjectMasking) {
    auto pixel{projection_functions::DoubleSphere::Project(double_sphere_intrinsics, testing_utilities::image_bounds,
                                                           {0, 0, 10})};
    EXPECT_TRUE(pixel.has_value());

    pixel = projection_functions::DoubleSphere::Project(double_sphere_intrinsics, testing_utilities::image_bounds,
                                                        {0, 0, -10});
    EXPECT_FALSE(pixel.has_value());
}

TEST(ProjectionFunctionsDoubleSphere, TestDoubleSphereUnproject) {
    auto const camera{
        projection_functions::DoubleSphereCamera(double_sphere_intrinsics, testing_utilities::image_bounds)};

    // NOTE(Jack): This is where the difference to a model like the pinhole model becomes apparent! For the pinhole
    // model all unprojected rays have a z-value of 1, and are somehow normalized in that sense. Here on the other hand,
    // for the double sphere model, all rays have a magnitude of one instead. Therefore instead of multiplying the rays
    // by the metric scale like we did for pinhole unprojection, we here instead normalize each point into
    // a ray with magnitude one.
    MatrixX3d const rays{camera.Unproject(gt_pixels)};
    MatrixX3d normalized_gt_points{testing_utilities::gt_points};
    normalized_gt_points.rowwise().normalize();

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}