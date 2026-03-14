#include "projection_functions/initialize_camera.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ProjectionFunctionsInitializeCamera, TestInitializeCamera) {
    ImageBounds const bounds{};

    auto camera_ptr{projection_functions::InitializeCamera(CameraModel::DoubleSphere, Array6d::Zero(), bounds)};
    EXPECT_TRUE(camera_ptr);

    camera_ptr = projection_functions::InitializeCamera(CameraModel::Pinhole, Array4d::Zero(), bounds);
    EXPECT_TRUE(camera_ptr);

    camera_ptr = projection_functions::InitializeCamera(CameraModel::PinholeRadtan4, Array8d::Zero(), bounds);
    EXPECT_TRUE(camera_ptr);

    camera_ptr = projection_functions::InitializeCamera(CameraModel::UnifiedCameraModel, Array5d::Zero(), bounds);
    EXPECT_TRUE(camera_ptr);
}
