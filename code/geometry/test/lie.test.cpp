#include "geometry/lie.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace reprojection::geometry;

std::vector<Eigen::Vector3d> const so3{Eigen::Vector3d{0, 0, 0},     //
                                       Eigen::Vector3d{M_PI, 0, 0},  //
                                       Eigen::Vector3d{0, M_PI, 0},  //
                                       Eigen::Vector3d{0, 0, M_PI}};

std::vector<Eigen::Matrix3d> const SO3{Eigen::Vector3d{1, 1, 1}.asDiagonal(),    //
                                       Eigen::Vector3d{1, -1, -1}.asDiagonal(),  //
                                       Eigen::Vector3d{-1, 1, -1}.asDiagonal(),  //
                                       Eigen::Vector3d{-1, -1, 1}.asDiagonal()};

TEST(GeometryLie, TestExp) {
    ASSERT_EQ(std::size(so3), std::size(SO3));

    for (size_t i{0}; i < std::size(so3); ++i) {
        Eigen::Vector3d const so3_i{so3[i]};
        Eigen::Matrix3d const SO3_i{Exp(so3_i)};

        EXPECT_TRUE(SO3_i.isApprox(SO3[i]));
    }
}

TEST(GeometryLie, TestLog) {
    ASSERT_EQ(std::size(so3), std::size(SO3));

    for (size_t i{0}; i < std::size(so3); ++i) {
        Eigen::Matrix3d const SO3_i{SO3[i]};
        Eigen::Vector3d const so3_i{Log(SO3_i)};

        EXPECT_TRUE(so3_i.isApprox(so3[i]));
    }
}

TEST(GeometryLie, TestInvertability) {
    Eigen::Vector3d const so3_random{M_PI * Eigen::Vector3d::Random()};
    Eigen::Vector3d const so3_random_processed{Log(Exp(so3_random))};
    EXPECT_TRUE(so3_random_processed.isApprox(so3_random));

    Eigen::Matrix3d const SO3_random{Eigen::Quaterniond::UnitRandom().toRotationMatrix()};
    Eigen::Matrix3d const SO3_random_processed{Exp(Log(SO3_random))};
    EXPECT_TRUE(SO3_random_processed.isApprox(SO3_random));
}
