#include "json_converters.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(DatabaseJsonConverters, TestDoubleSphere) {
    std::string const result{database::ToJson(CameraModel::DoubleSphere, Array6d{1, 2, 3, 4, 5, 6})};

    std::string const gt_result{"{\n    \"alpha\" : 6.0,\n    \"cx\" : 3.0,\n    \"cy\" : 4.0,\n    \"fx\" : 1.0,\n    \"fy\" : 2.0,\n    \"xi\" : 5.0\n}"};
    EXPECT_EQ(result, gt_result);
}