#include <gtest/gtest.h>

#include <Eigen/Dense>

// TODO THIS FUNCTION ACTUALLY BELONGS IN THE CORE PNP LOGIC TO DECIDE IF WE USE DLT OR HOMOGRAPHY DECOMPOSITION
bool IsPlane(Eigen::MatrixX3d const& points) {
    Eigen::Vector3d const center{points.colwise().mean()};
    Eigen::Matrix3d const covariance{(points.rowwise() - center.transpose()).transpose() *
                                     (points.rowwise() - center.transpose())};

    // TOOD(Jack): Are these reasonable svd computation options?
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Vector3d const singular_values{svd.singularValues()};
    if (singular_values[2] / singular_values[1] < 1e-3) {
        return true;
    }

    return false;
}

TEST(PnpHomographyDecomposition, TestXXX) {
    // Any three points are on a plane!
    Eigen::MatrixX3d const three_points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    EXPECT_TRUE(IsPlane(three_points));

    // z=0 plane
    Eigen::MatrixX3d plane{{1, 1, 0}, {2, 2, 0}, {3, 3, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}};
    EXPECT_TRUE(IsPlane(plane));

    // Make an outlier
    plane(0, 2) = 10;
    EXPECT_FALSE(IsPlane(plane));
}
