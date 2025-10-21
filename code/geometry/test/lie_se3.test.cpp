#include <gtest/gtest.h>

#include <vector>

#include "geometry/lie.hpp"

using namespace reprojection::geometry;

Eigen::Isometry3d ConstructTestSE3(Eigen::Vector3d const& R_diagonal, Eigen::Vector3d const& t) {
    Eigen::Isometry3d T{Eigen::Isometry3d::Identity()};
    T.linear() = R_diagonal.asDiagonal();
    T.translation() = t;

    return T;
}

std::vector<Eigen::Vector<double, 6>> const se3{Eigen::Vector<double, 6>{0, 0, 0, 0, 0, 0},     //
                                                Eigen::Vector<double, 6>{M_PI, 0, 0, 1, 0, 0},  //
                                                Eigen::Vector<double, 6>{0, M_PI, 0, 0, 1, 0},  //
                                                Eigen::Vector<double, 6>{0, 0, M_PI, 0, 0, 1}};

std::vector<Eigen::Isometry3d> const SE3{Eigen::Isometry3d::Identity(),             //
                                         ConstructTestSE3({1, -1, -1}, {1, 0, 0}),  //
                                         ConstructTestSE3({-1, 1, -1}, {0, 1, 0}),  //
                                         ConstructTestSE3({-1, -1, 1}, {0, 0, 1})};

TEST(GeometryLie, TestSe3Exp) {
    ASSERT_EQ(std::size(se3), std::size(SE3));

    for (size_t i{0}; i < std::size(se3); ++i) {
        Eigen::Vector<double, 6> const se3_i{se3[i]};
        Eigen::Isometry3d const SE3_i{Exp(se3_i)};

        EXPECT_TRUE(SE3_i.isApprox(SE3[i]));
    }
}

TEST(GeometryLie, TestSe3Log) {
    ASSERT_EQ(std::size(se3), std::size(SE3));

    for (size_t i{0}; i < std::size(se3); ++i) {
        Eigen::Isometry3d const SE3_i{SE3[i]};
        Eigen::Vector<double, 6> const se3_i{Log(SE3_i)};

        EXPECT_TRUE(se3_i.isApprox(se3[i]));
    }
}

TEST(GeometryLie, TestSe3Invertability) {
    Eigen::Vector<double, 6> const se3_random{M_PI * Eigen::Vector<double, 6>::Random()};
    Eigen::Vector<double, 6> const se3_random_processed{Log(Exp(se3_random))};
    EXPECT_TRUE(se3_random_processed.isApprox(se3_random));

    Eigen::Isometry3d SE3_random{Eigen::Quaterniond::UnitRandom().toRotationMatrix()};
    SE3_random.translation() = Eigen::Vector3d::Random();
    Eigen::Isometry3d const SE3_random_processed{Exp(Log(SE3_random))};
    EXPECT_TRUE(SE3_random_processed.isApprox(SE3_random));
}
