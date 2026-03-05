#include "serialize.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

// TODO(Jack): Fixture is copy and pasted
class SerializeFixture : public ::testing::Test {
   protected:
    // cppcheck-suppress-begin unusedStructMember
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    ExtractedTarget target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements camera_measurements{{0, target}, {1, target}};
    CameraState camera_state{testing_utilities::pinhole_intrinsics};
    Frames frames{{0, {Array6d::Ones()}}, {1, {2 * Array6d::Ones()}}};
    // cppcheck-suppress-end unusedStructMember
};

TEST_F(SerializeFixture, CameraInfo) {
    std::string const result{caching::Serialize(camera_info)};
    std::string const gt_result{"/cam/retro/123|pinhole|0.000,720.000,0.000,480.000|"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(SerializeFixture, CameraMeasurements) {
    std::string const result{caching::Serialize(camera_measurements)};
    std::string const gt_result{
        "0|1.230,1.430;2.750,2.350;|3.250,3.450,5.430;6.180,6.780,4.560;|5,6;2,3;|1|1.230,1.430;2.750,2.350;|3.250,3."
        "450,5.430;6.180,6.780,4.560;|5,6;2,3;|"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(SerializeFixture, CameraState) {
    std::string const result{caching::Serialize(camera_state)};
    std::string const gt_result{"600.000;600.000;360.000;240.000;|"};

    EXPECT_EQ(result, gt_result);
}

TEST_F(SerializeFixture, Frames) {
    std::string const result{caching::Serialize(frames)};
    std::string const gt_result{"0|1.000;1.000;1.000;1.000;1.000;1.000;|1|2.000;2.000;2.000;2.000;2.000;2.000;|"};

    EXPECT_EQ(result, gt_result);
}