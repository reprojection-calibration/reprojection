#pragma once

#include <gtest/gtest.h>

#include "april_tag_cpp_wrapper.hpp"
#include "target_generators.hpp"

extern "C" {
#include "generated_apriltag_code/tagCustom36h11.h"
}

namespace reprojection_calibration::feature_extraction {

class AprilTagTestFixture : public ::testing::Test {
   public:
    AprilTagTestFixture()
        : tag_family_handler_{AprilTagFamily{tagCustom36h11_create(), tagCustom36h11_destroy}},
          tag_detector_{AprilTagDetector{tag_family_handler_, {2.0, 0.0, 1, false, false}}},
          code_matrix_0_{AprilBoard3Generation::GenerateCodeMatrix(tag_family_handler_.tag_family->nbits,
                                                                   tag_family_handler_.tag_family->codes[0])},
          bit_size_pixel_{10} {}

    AprilTagFamily tag_family_handler_;
    AprilTagDetector tag_detector_;
    Eigen::MatrixXi code_matrix_0_;  // Arbitrarily choose the first code for testing
    int bit_size_pixel_;
};

}  // namespace reprojection_calibration::feature_extraction