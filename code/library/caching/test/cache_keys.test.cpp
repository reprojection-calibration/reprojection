#include "caching/cache_keys.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

// TODO(Jack): Fixture is copy and pasted
class CacheKeysFixture : public ::testing::Test {
   protected:
    // cppcheck-suppress-begin unusedStructMember
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    ExtractedTarget target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements camera_measurements{{0, target}, {1, target}};
    CameraState camera_state{testing_utilities::pinhole_intrinsics};
    Frames frames{{0, {Array6d::Ones()}}, {1, {2 * Array6d::Ones()}}};
    OptimizationState optimization_state{camera_state, frames};
    // cppcheck-suppress-end unusedStructMember
};

TEST_F(CacheKeysFixture, FocalLengthInitialization) {
    std::string const result{caching::CacheKey(camera_info, camera_measurements)};
    std::string const gt_result{"f4fa69df1a139cf559ab0e423312ecbcb1afe1fff9fffe0782da63a7c22e1b51"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(CacheKeysFixture, LinearPoseInitialization) {
    std::string const result{caching::CacheKey(camera_info, camera_measurements, camera_state)};
    std::string const gt_result{"c78e49018ab68ffc6b2ce19cc210aaba72ac7729ce5385ef9a231a34af2032fe"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(CacheKeysFixture, CameraNonlinearRefinement) {
    std::string const result{caching::CacheKey(camera_info, camera_measurements, optimization_state)};
    std::string const gt_result{"b45889d48b71fc02fd39a3d8f9a41a909d6e98627d491596a63b2e24b2dca5b8"};

    EXPECT_EQ(result, gt_result);
}