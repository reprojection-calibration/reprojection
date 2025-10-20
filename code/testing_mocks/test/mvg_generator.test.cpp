#include <gtest/gtest.h>

#include <fstream>   // SAVING POINTS REMOVE
#include <iostream>  // SAVING POINTS REMOVE

#include "constants.hpp"
#include "geometry/lie.hpp"
#include "sphere_trajectory.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

// Function to save a vector of Eigen::Isometry3d transformations to a CSV file
void saveTransformsToCSV(const std::vector<Eigen::Isometry3d>& transforms, const std::string& filename) {
    // Open the file to write the matrix
    std::ofstream file(filename);

    // Check if the file is open
    if (file.is_open()) {
        // Iterate over each transform in the vector
        for (const auto& transform : transforms) {
            // Write the 4x4 matrix of each transform on a new row
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    file << transform.matrix()(i, j);
                    if (not(i * j == 9)) file << ",";  // Add a comma if not the last column
                }
            }
            file << "\n";  // Newline after tf
        }
        std::cout << "Transformation matrices saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file " << filename << "!" << std::endl;
    }
}

// COPIED FROM FEATURE EXTRACTION
Eigen::ArrayXi ToEigen(std::vector<int> const& vector) {
    return Eigen::Map<Eigen::ArrayXi const>(vector.data(), std::size(vector));
}

// COPIED FROM FEATURE EXTRACTION
// There has to be a more eloquent way to do this... but it gets the job done :)
Eigen::ArrayXi MaskIndices(Eigen::ArrayXi const& array) {
    std::vector<int> mask;
    mask.reserve(array.rows());

    for (Eigen::Index i{0}; i < array.rows(); i++) {
        if (array(i) == 1) {
            mask.push_back(i);
        }
    }

    return ToEigen(mask);
}

// COPIED FROM FEATURE EXTRACTION
Eigen::ArrayX2i GenerateGridIndices(int const rows, int const cols, bool const even_only) {
    Eigen::ArrayXi const row_indices{Eigen::ArrayXi::LinSpaced(rows * cols, 0, rows - 1)};
    Eigen::ArrayXi const col_indices{Eigen::ArrayXi::LinSpaced(cols, 0, cols).colwise().replicate(rows)};

    Eigen::ArrayX2i grid_indices(rows * cols, 2);
    grid_indices.col(0) = row_indices;
    grid_indices.col(1) = col_indices;

    if (even_only) {
        // NOTE(Jack): Eigen does not provide direct way to apply the modulo operator, so we follow a method using a
        // unaryExpr() that we adopted from here
        // (https://stackoverflow.com/questions/35798698/eigen-matrix-library-coefficient-wise-modulo-operation)
        Eigen::ArrayXi const is_even{
            ((grid_indices.rowwise().sum().unaryExpr([](int const x) { return x % 2; })) == 0).cast<int>()};
        Eigen::ArrayXi const mask{MaskIndices(is_even)};

        return grid_indices(mask, Eigen::all);
    }

    return grid_indices;
}

struct MvgFrame {
    Eigen::Isometry3d pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

class MvgGenerator {
   public:
    MvgGenerator(bool const flat = true,
                 Eigen::Matrix3d const& K = Eigen::Matrix3d{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}})
        : K_{K}, se3_spline_{constants::t0_ns, constants::delta_t_ns} {
        CameraTrajectory const config{{0, 0, 0}, 1.0, {1, 1, 5}};
        std::vector<Eigen::Isometry3d> const poses{SphereTrajectory(config)};

        // NOTE(Jack): To get to the actual ends of the sphere on the spline we would need to have one knot before the
        // start and three knots past the end of the sphere to allow for interpolation to work (for spline degree=4).
        // Here we do not have that.
        for (auto const& pose : poses) {
            se3_spline_.AddKnot(pose);
        }

        int const rows{5};
        int const cols{rows};
        int const num_points{rows * cols};
        Eigen::ArrayX2i const grid{GenerateGridIndices(rows, cols, false)};
        points_ = Eigen::MatrixX3d::Zero(num_points, 3);
        points_.leftCols(2) = grid.cast<double>();
        // Center around zero with unit range [-0.5, 0.5]
        points_.col(0) = points_.col(0) / (rows - 1);
        points_.col(0) = points_.col(0).array() - 0.5;
        points_.col(1) = points_.col(1) / (cols - 1);
        points_.col(1) = points_.col(1).array() - 0.5;

        if (not flat) {
            // Add z-axis values in range [-0.5, 0.5]
            points_.col(2) = Eigen::VectorXd::Random(num_points) / 2;
        }
    }

    // Input is fractional time of trajectory from [0,1)
    MvgFrame Generate(double const t) const {
        assert( 0 <=t and t < 1);

        // TODO(Jack): Check boundary conditions!
        // Static cast means we loose some precision, but at nanosecond level this should not matter.
        uint64_t const spline_time{constants::t0_ns +
                                   static_cast<uint64_t>((constants::num_poses * constants::delta_t_ns) * t)};

        // WARN(Jack): This returns an optional but we do not check it! Assuming we calculated spline_time correctly and
        // t is [0,1) then this should be no problem, but this will be a problem :)
        auto const pose_t{se3_spline_.Evaluate(spline_time)};

        Eigen::MatrixX2d const pixels{Project(points_, K_, pose_t.value())};

        // WARN(Jack): This assumes that all points are always visible! With careful engineering for the default value
        // of K this will be true, but that cannot be guarantted for all K!!!
        return {pose_t.value(), pixels, points_};
    }

    static Eigen::MatrixX2d Project(Eigen::MatrixX3d const& points_w, Eigen::Matrix3d const& K,
                                    Eigen::Isometry3d const& tf_co_w) {
        // TODO(Jack): Do we need to transform isometries into matrices before we use them? Otherwise it might not
        // match our expectations about matrix dimensions after the fact.
        // TODO(Jack): Should we use the pinhole projection from the nonlinear refinement optimization here?
        Eigen::MatrixX4d const points_homog_co{(tf_co_w * points_w.rowwise().homogeneous().transpose()).transpose()};
        Eigen::MatrixX2d const pixels{
            (K * points_homog_co.leftCols(3).transpose()).transpose().rowwise().hnormalized()};

        return pixels;
    }

   private:
    Eigen::Matrix3d K_;
    spline::Se3Spline se3_spline_;
    Eigen::MatrixX3d points_;
};

}  // namespace reprojection::testing_mocks

using namespace reprojection::testing_mocks;

TEST(TestingMocks, XXX) {
    auto const generator{MvgGenerator()};

    auto const frame{generator.Generate(0.5)};

    std::cout << frame.pixels << std::endl;

    EXPECT_FALSE(true);
}