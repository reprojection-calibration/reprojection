#include "hashing/hashing.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(CachingHashing, TestHash) {
    std::string const result{hashing::Sha256("Jack")};
    EXPECT_EQ(result, "b5fd03dd91df1cfbd2f19c115d24d58bbda01a23fb01924bb78b2cc14f7ff1cb");
}

// TODO(Jack): Fixture is copy and pasted
class HashingFixture : public ::testing::Test {
   protected:
    // cppcheck-suppress-begin unusedStructMember
    CameraInfo camera_info{CameraModel::Pinhole, testing_utilities::image_bounds};
    ExtractedTarget target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    TargetSamples camera_measurements{{0, target}, {1, target}};
    Intrinsic camera_state{testing_utilities::pinhole_intrinsics};
    Frames frames{{0, {Array6d::Ones()}}, {1, {2 * Array6d::Ones()}}};
    OptimizationState optimization_state{camera_state, frames};
    // cppcheck-suppress-end unusedStructMember
};

TEST_F(HashingFixture, FocalLengthInitialization) {
    Hash const result{hashing::HashArguments(camera_info, camera_measurements)};
    Hash const gt_result{"430782805bd5d77fa692e90432f3d6ae6cc997c75896ada95d1db828601f3e17"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(HashingFixture, PoseInitialization) {
    Hash const result{hashing::HashArguments(camera_info, camera_measurements, camera_state)};
    Hash const gt_result{"f1e08743677a82f725c0f7c5125bbf3819121e1c0ce6222106abaae5d71726c9"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(HashingFixture, BundleAdjustment) {
    Hash const result{hashing::HashArguments(camera_info, camera_measurements, optimization_state)};
    Hash const gt_result{"27ef005062911d3fcdb3bf1b0b8d8bb6f5900f0b05b994ce409e1573903b1e1d"};

    EXPECT_EQ(result, gt_result);
}