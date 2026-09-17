#include "intrinsic_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"

using namespace reprojection;

TEST(CalibrationIntrinsicInit, TestSelectInitializationStrategy) {
    double f{};
    double height{};
    double width{};

    auto intrinsic_init{calibration::SelectInitializationStrategy(CameraModel::DoubleSphere)};
    ArrayXd intrinsic{intrinsic_init(f, height, width)};
    EXPECT_EQ(intrinsic.size(), 5);

    intrinsic_init = calibration::SelectInitializationStrategy(CameraModel::Eucm);
    intrinsic = intrinsic_init(f, height, width);
    EXPECT_EQ(intrinsic.size(), 5);

    intrinsic_init = calibration::SelectInitializationStrategy(CameraModel::Pinhole);
    intrinsic = intrinsic_init(f, height, width);
    EXPECT_EQ(intrinsic.size(), 3);

    intrinsic_init = calibration::SelectInitializationStrategy(CameraModel::PinholeRadtan4);
    intrinsic = intrinsic_init(f, height, width);
    EXPECT_EQ(intrinsic.size(), 7);

    intrinsic_init = calibration::SelectInitializationStrategy(CameraModel::Ucm);
    intrinsic = intrinsic_init(f, height, width);
    EXPECT_EQ(intrinsic.size(), 4);
}
