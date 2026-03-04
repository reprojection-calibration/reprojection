#include "caching/cache_keys.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(CachingCacheKeys, FocalLengthInitialization) {
    CameraInfo const camera_info{"/cam/retro/123", CameraModel::DoubleSphere, testing_utilities::image_bounds};
    ExtractedTarget const target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements const camera_measurements{{0, target}, {1, target}};

    std::string const result{caching::CacheKey(camera_info, camera_measurements)};
    std::string const gt_result{"696a97934101f9d594dd9645ef1d68455920eca08c8b9b940ef5e77b7e984b51"};

    EXPECT_EQ(result, gt_result);
}

TEST(CachingCacheKeys, LinearPoseInitialization) {
    CameraInfo const camera_info{"/cam/retro/123", CameraModel::DoubleSphere, testing_utilities::image_bounds};
    ExtractedTarget const target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements const camera_measurements{{0, target}, {1, target}};
    CameraState const camera_state{testing_utilities::pinhole_intrinsics};

    std::string const result{caching::CacheKey(camera_info, camera_measurements, camera_state)};
    std::string const gt_result{"34ffa0d68b2fc52ce58413bb6b75796af57eac23d5a893d5f94c6aa64de6ae16"};

    EXPECT_EQ(result, gt_result);
}

// TODO(Jack): We need some test fixtures! The testing boilerplate is getting out of hand!
TEST(CachingCacheKeys, NonlinearCameraRefinement) {
    CameraInfo const camera_info{"/cam/retro/123", CameraModel::DoubleSphere, testing_utilities::image_bounds};
    ExtractedTarget const target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements const camera_measurements{{0, target}, {1, target}};
    CameraState const camera_state{testing_utilities::pinhole_intrinsics};
    Frames const frames{{0, {Array6d::Ones()}}, {1, {2 * Array6d::Ones()}}};
    OptimizationState const optimization_state{camera_state, frames};

    std::string const result{caching::CacheKey(camera_info, camera_measurements, optimization_state)};
    std::string const gt_result{"d82a4f009e0c90531726608bcd5c52c1952ada5d0f3de04eb625cb3a48f87da0"};

    EXPECT_EQ(result, gt_result);
}