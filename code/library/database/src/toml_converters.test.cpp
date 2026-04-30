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

TEST(DatabaseTomlConverters, TestFromToml) {
    std::string data{"alpha = 6.1\ncx = 3.1\ncy = 4.1\nfx = 1.1\nfy = 2.1\nxi = 5.1"};
    Array6d const ds_result{database::FromToml(CameraModel::DoubleSphere, data)};
    Array6d const gt_ds_result{{1.1, 2.1, 3.1, 4.1, 5.1, 6.1}};
    EXPECT_TRUE(ds_result.isApprox(gt_ds_result));

    data = std::string{"cx = 3.2\ncy = 4.2\nfx = 1.2\nfy = 2.2"};
    Array4d const ph_result{database::FromToml(CameraModel::Pinhole, data)};
    Array4d const gt_ph_result{1.2, 2.2, 3.2, 4.2};
    EXPECT_TRUE(ph_result.isApprox(gt_ph_result));

    data = std::string{"cx = 3.3\ncy = 4.3\nfx = 1.3\nfy = 2.3\nk1 = 5.3\nk2 = 6.3\np1 = 7.3\np2 = 8.3"};
    Array8d const phrt4_result{database::FromToml(CameraModel::PinholeRadtan4, data)};
    Array8d const gt_phrt4_result{1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3, 8.3};
    EXPECT_TRUE(phrt4_result.isApprox(gt_phrt4_result));

    data = std::string{"cx = 3.4\ncy = 4.4\nfx = 1.4\nfy = 2.4\nxi = 5.4"};
    Array5d const ucm_result{database::FromToml(CameraModel::UnifiedCameraModel, data)};
    Array5d const gt_ucm_result{1.4, 2.4, 3.4, 4.4, 5.4};
    EXPECT_TRUE(ucm_result.isApprox(gt_ucm_result));
}