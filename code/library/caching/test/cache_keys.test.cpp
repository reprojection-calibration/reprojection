#include "caching/cache_keys.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

// TODO(Jack): Fixture is copy and pasted
class CachingFixture : public ::testing::Test {
   protected:
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    ExtractedTarget target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements camera_measurements{{0, target}, {1, target}};
    CameraState camera_state{testing_utilities::pinhole_intrinsics};
    Frames frames{{0, {Array6d::Ones()}}, {1, {2 * Array6d::Ones()}}};
    OptimizationState optimization_state{camera_state, frames};
};

TEST_F(CachingFixture, FocalLengthInitialization) {
    std::string const result{caching::CacheKey(camera_info, camera_measurements)};
    std::string const gt_result{"f4fa69df1a139cf559ab0e423312ecbcb1afe1fff9fffe0782da63a7c22e1b51"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(CachingFixture, LinearPoseInitialization) {
    std::string const result{caching::CacheKey(camera_info, camera_measurements, camera_state)};
    std::string const gt_result{"947b431f312e71ed52b4e85b84ae36e34c62ac1dd749f3988f631d23f3c029b9"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(CachingFixture, CameraNonlinearRefinement) {
    std::string const result{caching::CacheKey(camera_info, camera_measurements, optimization_state)};
    std::string const gt_result{"2a49e05662f0f402884fcb00bbe9b3388979381e9324181e513c56edd255ced7"};

    EXPECT_EQ(result, gt_result);
}