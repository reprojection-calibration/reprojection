#include "calibration/focal_length_initialization.hpp"

#include <gtest/gtest.h>

#include "eigen_utilities/grid.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(CalibrationFocalLengthInitialization, TestParabolaLine) {

    ArrayX2i const indices{eigen_utilities::GenerateGridIndices(6, 7)};

    Eigen::ArrayXXd result(indices.rows(), 3);
    result << indices.cast<double>(), Eigen::ArrayXd::Constant(indices.rows(), 0);

    std::cout << result<<std::endl;

    //auto const camera{
    //    projection_functions::PinholeCamera(testing_utilities::double_sphere_intrinsics, testing_utilities::image_bounds)};


}