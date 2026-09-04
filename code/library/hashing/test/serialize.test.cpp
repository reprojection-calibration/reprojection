#include "hashing/serialize.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(HashingSerialize, TestSerializeCameraInfo) {
    CameraInfo const camera_info{CameraModel::Pinhole, testing_utilities::image_bounds};

    std::string const result{hashing::Serialize(camera_info)};
    std::string const gt_result{"pinhole|0.000,720.000,0.000,480.000|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeCameraMeasurements) {
    ExtractedTarget const target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements const camera_measurements{{0, target}, {1, target}};

    std::string const result{hashing::Serialize(camera_measurements)};
    std::string const gt_result{
        "0|1.230,1.430;2.750,2.350;|3.250,3.450,5.430;6.180,6.780,4.560;|5,6;2,3;|1|1.230,1.430;2.750,2.350;|3.250,3."
        "450,5.430;6.180,6.780,4.560;|5,6;2,3;|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeCameraState) {
    Intrinsic const camera_state{testing_utilities::pinhole_intrinsics};

    std::string const result{hashing::Serialize(camera_state)};
    std::string const gt_result{"600.000;360.000;240.000;|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeControlPointMatrix) {
    Eigen::Matrix<double, 6, 2> const control_points{{1, 2}, {3, 4}, {5, 6}, {1, 2}, {3, 4}, {5, 6}};

    std::string const result{hashing::Serialize(control_points)};
    std::string const gt_result{"1.000,2.000;3.000,4.000;5.000,6.000;1.000,2.000;3.000,4.000;5.000,6.000;"};

    EXPECT_EQ(result, gt_result);
}
TEST(HashingSerialize, TestSerializeEncodedImages) {
    EncodedImages const encoded_images{{0, ImageBuffer{}}, {1, ImageBuffer{}}};

    std::string const result{hashing::Serialize(encoded_images)};
    std::string const gt_result{"0|0|1|0|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeExtrinsic) {
    Extrinsic const data{AssetId{1}, AssetId{2}, Array6d::Ones()};

    std::string const result{hashing::Serialize(data)};
    std::string const gt_result{"1|2|1.000;1.000;1.000;1.000;1.000;1.000;|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeFrames) {
    Frames const frames{{0, {Array6d::Ones()}}, {1, {2 * Array6d::Ones()}}};

    std::string const result{hashing::Serialize(frames)};
    std::string const gt_result{"0|1.000;1.000;1.000;1.000;1.000;1.000;|1|2.000;2.000;2.000;2.000;2.000;2.000;|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeImuMeasurements) {
    ImuMeasurements const imu_data{{0, {{0, 1, 2}, {3, 4, 5}}}};

    std::string const result{hashing::Serialize(imu_data)};
    std::string const gt_result{"0|0.000;1.000;2.000;|3.000;4.000;5.000;|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestSerializeConfigTarget) {
    config::Config::Target const target_info{TargetType::Aprilgrid3, {8, 6}, 0.1, false};

    std::string const result{hashing::Serialize(target_info)};
    std::string const gt_result{"aprilgrid3|8,6|0.100|0|"};

    EXPECT_EQ(result, gt_result);
}

TEST(HashingSerialize, TestAssetsVector) {
    std::vector<AssetId> const data{{0}, {1}, {5}};

    std::string const result{hashing::Serialize(data)};
    std::string const gt_result{"0|1|5|"};

    EXPECT_EQ(result, gt_result);
}