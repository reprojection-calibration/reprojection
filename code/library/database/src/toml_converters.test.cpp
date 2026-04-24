#include "toml_converters.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(DatabaseTomlConverters, TestToToml) {
    std::string result{database::ToToml(CameraModel::DoubleSphere, Array6d{1, 2, 3, 4, 5, 6})};
    std::string gt_result{"alpha = 6.0\ncx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0\nxi = 5.0"};
    EXPECT_EQ(result, gt_result);

    result = database::ToToml(CameraModel::Pinhole, Array4d{1, 2, 3, 4});
    gt_result = "cx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0";
    EXPECT_EQ(result, gt_result);

    result = database::ToToml(CameraModel::PinholeRadtan4, Eigen::Array<double, 8, 1>{1, 2, 3, 4, 5, 6, 7, 8});
    gt_result = "cx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0\nk1 = 5.0\nk2 = 6.0\np1 = 7.0\np2 = 8.0";
    EXPECT_EQ(result, gt_result);

    result = database::ToToml(CameraModel::UnifiedCameraModel, Array5d{1, 2, 3, 4, 5});
    gt_result = "cx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0\nxi = 5.0";
    EXPECT_EQ(result, gt_result);
}

// TODO(Jack): Add tests for pinhole.
TEST(DatabaseTomlConverters, TestFromToml) {
    std::string data{"alpha = 6.0\ncx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0\nxi = 5.0"};
    ArrayXd result{database::FromToml(CameraModel::DoubleSphere, data)};

    ArrayXd gt_result{Array6d{1, 2, 3, 4, 5, 6}};
    EXPECT_TRUE(result.isApprox(gt_result));

    data = std::string{"cx = 1.0\ncy = 2.0\nfx = 3.0\nfy = 4.0\nk1 = 5.0\nk2 = 6.0\np1 = 7.0\np2 = 8.0"};
    result = database::FromToml(CameraModel::PinholeRadtan4, data);

    gt_result = Array8d{3, 4, 1, 2, 5, 6, 7, 8};
    EXPECT_TRUE(result.isApprox(gt_result));
}