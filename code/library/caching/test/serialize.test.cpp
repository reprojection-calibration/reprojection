#include "caching/serialize.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(CachingSerialize, CameraMeasurements) {
    ExtractedTarget const target{
        Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}}},
        {{5, 6}, {2, 3}}};
    CameraMeasurements const data{{0, target}, {1, target}};

    std::string const result{caching::Serialize(data)};

    std::string const gt_result{
        "0|1.230,1.430;2.750,2.350;|3.250,3.450,5.430;6.180,6.780,4.560;|5,6;2,3;|1|1.230,1.430;2.750,2.350;|3.250,3."
        "450,5.430;6.180,6.780,4.560;|5,6;2,3;|"};

    EXPECT_EQ(result, gt_result);
}