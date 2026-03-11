#include "calibration/focal_length_initialization.hpp"

#include <gtest/gtest.h>

#include "eigen_utilities/grid.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(CalibrationFocalLengthInitialization, TestParabolaLine) {
    ArrayX2i const indices{eigen_utilities::GenerateGridIndices(6, 7)};
    Eigen::ArrayXXd target_points(indices.rows(), 3);
    target_points << indices.cast<double>(), Eigen::ArrayXd::Constant(indices.rows(), 15);

    Array5d const intrinsics{600, 600, 360, 240, 1};
    auto const camera{projection_functions::UcmCamera(intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask]{camera.Project(target_points)};
    ASSERT_TRUE(mask.all());

    ExtractedTarget const target{{pixels, target_points}, indices};
    auto const result{calibration::InitializeFocalLengthParabolaLine(target, intrinsics.segment(2, 2))};

    EXPECT_EQ(std::size(result), 9);  // Arbitrary number of successful initializations
    for (auto const& f_i : result) {
        EXPECT_FLOAT_EQ(f_i, 600);
    }
}

TEST(CalibrationFocalLengthInitialization, TestInitializeIntrinsics) {
    ArrayX2i const indices{eigen_utilities::GenerateGridIndices(6, 7)};
    Eigen::ArrayXXd target_points(indices.rows(), 3);
    target_points << indices.cast<double>(), Eigen::ArrayXd::Constant(indices.rows(), 15);

    // NOTE(Jack): At time of writing we only have initialization logic for ds model therefore we use it here.
    Array6d const intrinsics{testing_utilities::double_sphere_intrinsics};
    auto const camera{projection_functions::DoubleSphereCamera(intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask]{camera.Project(target_points)};
    ASSERT_TRUE(mask.all());

    ExtractedTarget const target{{pixels, target_points}, indices};

    auto const gamma{calibration::InitializeIntrinsics<projection_functions::DoubleSphere>(
        {{0, target}}, testing_utilities::image_bounds.v_max, testing_utilities::image_bounds.u_max)};

    ASSERT_TRUE(gamma.has_value());
    EXPECT_FLOAT_EQ(*gamma, 1037.1425);
}