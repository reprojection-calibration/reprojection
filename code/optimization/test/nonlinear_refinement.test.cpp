#include "optimization/nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// TODO(Jack): Test the nonlinear refinement with noisy data to make sure the optimization executes more than one step!

TEST(OptimizationNonlinearRefinement, TestNonlinearRefinement) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        auto const [tf, K]{optimization::NonlinearRefinement(frame_i.pixels, frame_i.points, frame_i.pose,
                                                             CameraModel::Pinhole, intrinsics)};

        EXPECT_TRUE(tf.isApprox(frame_i.pose)) << "Optimization result:\n"
                                               << geometry::Log(tf) << "\noptimization input:\n"
                                               << geometry::Log(frame_i.pose);
        EXPECT_TRUE(K.isApprox(intrinsics)) << "Optimization result:\n" << K << "\noptimization input:\n" << intrinsics;
    }
}
