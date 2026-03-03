#include "caching/cache_keys.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

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