#include <gtest/gtest.h>

#include <vector>

#include "geometry/lie.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

std::vector<Vector3d> const so3{Vector3d{0, 0, 0},     //
                                Vector3d{M_PI, 0, 0},  //
                                Vector3d{0, M_PI, 0},  //
                                Vector3d{0, 0, M_PI}};

std::vector<Matrix3d> const SO3{Vector3d{1, 1, 1}.asDiagonal(),    //
                                Vector3d{1, -1, -1}.asDiagonal(),  //
                                Vector3d{-1, 1, -1}.asDiagonal(),  //
                                Vector3d{-1, -1, 1}.asDiagonal()};

TEST(GeometryLie, TestSo3Exp) {
    ASSERT_EQ(std::size(so3), std::size(SO3));

    for (size_t i{0}; i < std::size(so3); ++i) {
        Vector3d const so3_i{so3[i]};
        Matrix3d const SO3_i{geometry::Exp(so3_i)};

        EXPECT_TRUE(SO3_i.isApprox(SO3[i]));
    }
}

TEST(GeometryLie, TestSo3Log) {
    ASSERT_EQ(std::size(so3), std::size(SO3));

    for (size_t i{0}; i < std::size(so3); ++i) {
        Matrix3d const SO3_i{SO3[i]};
        Vector3d const so3_i{geometry::Log(SO3_i)};

        EXPECT_TRUE(so3_i.isApprox(so3[i]));
    }
}

TEST(GeometryLie, TestSo3Invertability) {
    Vector3d const so3_random{M_PI * Vector3d::Random()};
    Vector3d const so3_random_processed{geometry::Log(geometry::Exp(so3_random))};
    EXPECT_TRUE(so3_random_processed.isApprox(so3_random));

    Matrix3d const SO3_random{Eigen::Quaterniond::UnitRandom().toRotationMatrix()};
    Matrix3d const SO3_random_processed{geometry::Exp(geometry::Log(SO3_random))};
    EXPECT_TRUE(SO3_random_processed.isApprox(SO3_random));
}
