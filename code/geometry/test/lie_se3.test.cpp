#include <gtest/gtest.h>

#include <vector>

#include "geometry/lie.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Isometry3d ConstructTestSE3(Vector3d const& R_diagonal, Vector3d const& t) {
    Isometry3d T{Isometry3d::Identity()};
    T.linear() = R_diagonal.asDiagonal();
    T.translation() = t;

    return T;
}

std::vector<Vector6d> const se3{Vector6d{0, 0, 0, 0, 0, 0},     //
                                Vector6d{M_PI, 0, 0, 1, 0, 0},  //
                                Vector6d{0, M_PI, 0, 0, 1, 0},  //
                                Vector6d{0, 0, M_PI, 0, 0, 1}};

std::vector<Isometry3d> const SE3{Isometry3d::Identity(),                    //
                                  ConstructTestSE3({1, -1, -1}, {1, 0, 0}),  //
                                  ConstructTestSE3({-1, 1, -1}, {0, 1, 0}),  //
                                  ConstructTestSE3({-1, -1, 1}, {0, 0, 1})};

TEST(GeometryLie, TestSe3Exp) {
    ASSERT_EQ(std::size(se3), std::size(SE3));

    for (size_t i{0}; i < std::size(se3); ++i) {
        Vector6d const se3_i{se3[i]};
        Isometry3d const SE3_i{geometry::Exp(se3_i)};

        EXPECT_TRUE(SE3_i.isApprox(SE3[i]));
    }
}

TEST(GeometryLie, TestSe3Log) {
    ASSERT_EQ(std::size(se3), std::size(SE3));

    for (size_t i{0}; i < std::size(se3); ++i) {
        Isometry3d const SE3_i{SE3[i]};
        Vector6d const se3_i{geometry::Log(SE3_i)};

        EXPECT_TRUE(se3_i.isApprox(se3[i]));
    }
}

TEST(GeometryLie, TestSe3Invertability) {
    Vector6d const se3_random{M_PI * Vector6d::Random()};
    Vector6d const se3_random_processed{geometry::Log(geometry::Exp(se3_random))};
    EXPECT_TRUE(se3_random_processed.isApprox(se3_random));

    Isometry3d SE3_random{Eigen::Quaterniond::UnitRandom().toRotationMatrix()};
    SE3_random.translation() = Vector3d::Random();
    Isometry3d const SE3_random_processed{geometry::Exp(geometry::Log(SE3_random))};
    EXPECT_TRUE(SE3_random_processed.isApprox(SE3_random));
}
