#include "json_converters.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(DatabaseJsonConverters, TestToJson) {
    std::string const result{database::ToToml(CameraModel::DoubleSphere, Array6d{1, 2, 3, 4, 5, 6})};

    std::string const gt_result{"alpha = 6.0\ncx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0\nxi = 5.0"};
    EXPECT_EQ(result, gt_result);
}

TEST(DatabaseJsonConverters, TestFromJson) {
    std::string const data{"alpha = 6.0\ncx = 3.0\ncy = 4.0\nfx = 1.0\nfy = 2.0\nxi = 5.0"};
    ArrayXd const result{database::FromToml(CameraModel::DoubleSphere, data)};

    Array6d const gt_result{1, 2, 3, 4, 5, 6};
    EXPECT_TRUE(result.isApprox(gt_result));
}