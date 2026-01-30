#include "mvg_helpers.hpp"

#include "eigen_utilities/grid.hpp"

namespace reprojection::testing_mocks {

/**
 * \brief Static helper method that projects points in the world frame. Do NOT use outside the testing mocks context!
 *
 * This method is intended only for use as part of the testing mocks test data generation class
 * (reprojection::testing_mocks::MvgHelpers) and should NOT be used by other consuming code. It was left as a public
 * method only so that it could be tested.
 */
std::tuple<MatrixX2d, ArrayXb> MvgHelpers::Project(MatrixX3d const& points_w,
                                                   std::unique_ptr<projection_functions::Camera> const& camera,
                                                   Isometry3d const& tf_co_w) {
    MatrixX4d const points_homog_co{(tf_co_w * points_w.rowwise().homogeneous().transpose()).transpose()};

    return camera->Project(points_homog_co.leftCols(3));
}

MatrixX3d MvgHelpers::BuildTargetPoints(bool const flat) {
    int const size{5};  // Square target - rows == cols
    int const num_points{size * size};

    ArrayX2i const grid{eigen_utilities::GenerateGridIndices(size, size, false)};
    MatrixX3d points{MatrixX3d::Zero(num_points, 3)};

    // Center around zero with unit range [-0.5, 0.5]
    points.leftCols(2) = grid.cast<double>();
    points.leftCols(2).array() /= (size - 1);
    points.leftCols(2).array() -= 0.5;

    if (not flat) {
        points.col(2) = VectorXd::Random(num_points) / 2;  // Add random z-axis values in range [-0.5, 0.5]
    }

    return points;
}

}  // namespace reprojection::testing_mocks