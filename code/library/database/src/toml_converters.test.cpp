#include "toml_converters.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"

using namespace reprojection;

std::string const pinhole_toml{"cx = 360.0\ncy = 240.0\nf = 600.0"};
std::string const ds_toml{"alpha = 0.20000000000000001\n" + pinhole_toml + "\nxi = 0.10000000000000001"};
std::string const pinhole_radtan4_toml{pinhole_toml + "\nk1 = 1.0\nk2 = 2.0\np1 = 3.0\np2 = 4.0"};
std::string const ucm_toml{pinhole_toml + "\nxi = 5.0"};

// NOTE(Jack): This lambda initialization maybe looks a little ugly but this is the only direct way to define this
// global here. We could also do this inside a test fixture, but I think that adds accidental complexity. Our solution
// here, given the constraints of eigen array initialization, is just the essential complexity.
Array7d const pinhole_radtan4_intrinsics{[]() {
    Array7d data;
    data << testing_utilities::pinhole_intrinsics, 1, 2, 3, 4;

    return data;
}()};

Array4d const ucm_intrinsics{[]() {
    Array4d data;
    data << testing_utilities::pinhole_intrinsics, 5;

    return data;
}()};

// TODO(Jack): We can also replace the two following tests with a single test that checks the forward and reverse
// operation in one line.

TEST(DatabaseTomlConverters, TestToToml) {
    std::string result{database::ToToml(CameraModel::DoubleSphere, testing_utilities::double_sphere_intrinsics)};
    EXPECT_EQ(result, ds_toml);

    result = database::ToToml(CameraModel::Pinhole, testing_utilities::pinhole_intrinsics);
    EXPECT_EQ(result, pinhole_toml);

    result = database::ToToml(CameraModel::PinholeRadtan4, pinhole_radtan4_intrinsics);
    EXPECT_EQ(result, pinhole_radtan4_toml);

    result = database::ToToml(CameraModel::UnifiedCameraModel, ucm_intrinsics);
    EXPECT_EQ(result, ucm_toml);
}

TEST(DatabaseTomlConverters, TestFromToml) {
    Array5d const ds_result{database::FromToml(CameraModel::DoubleSphere, ds_toml)};
    EXPECT_TRUE(ds_result.isApprox(testing_utilities::double_sphere_intrinsics));

    Array3d const pinhole_result{database::FromToml(CameraModel::Pinhole, pinhole_toml)};
    EXPECT_TRUE(pinhole_result.isApprox(testing_utilities::pinhole_intrinsics));

    Array7d const pinhole_radtan4_result{database::FromToml(CameraModel::PinholeRadtan4, pinhole_radtan4_toml)};
    EXPECT_TRUE(pinhole_radtan4_result.isApprox(pinhole_radtan4_intrinsics));

    Array4d const ucm_result{database::FromToml(CameraModel::UnifiedCameraModel, ucm_toml)};
    EXPECT_TRUE(ucm_result.isApprox(ucm_intrinsics));
}