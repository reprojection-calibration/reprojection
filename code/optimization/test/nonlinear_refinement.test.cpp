#include "optimization/nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// TODO(Jack): Test the nonlinear refinement with noisy data to make sure the optimization executes more than one step!

TEST(OptimizationNonlinearRefinement, TestNonlinearRefinementBatch) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    int const num_frames{20};
    std::vector<Frame> const frames{generator.GenerateBatchFrames(num_frames)};

    auto const [poses_opt, K]{optimization::NonlinearRefinement(frames, CameraModel::Pinhole, intrinsics)};

    for (size_t i{0}; i < std::size(poses_opt); ++i) {
        auto const pose_opt_i{poses_opt[i]};
        auto const pose_i{frames[i].pose};

        EXPECT_TRUE(pose_opt_i.isApprox(pose_i, 1e-6)) << "Optimization result:\n"
                                                       << geometry::Log(pose_opt_i).transpose() << "\nGround truth:\n"
                                                       << geometry::Log(pose_i).transpose();
    }

    EXPECT_TRUE(K.isApprox(intrinsics, 1e-6)) << "Optimization result:\n"
                                              << K << "\noptimization input:\n"
                                              << intrinsics;
}

// TODO(Jack): DO WE REALLY NEED THIS TEST IF WE HAVE THE BATCH TEST AND ALSO THIS GETS TESTED IN THE PNP CODE
TEST(OptimizationNonlinearRefinement, TestNonlinearRefinement) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    std::vector<Frame> const frames{generator.GenerateBatchFrames(20)};
    for (auto const& frame : frames) {
        auto const [tf, K]{optimization::NonlinearRefinement({frame}, CameraModel::Pinhole, intrinsics)};
        Isometry3d const tf_i{tf[0]};  // Only one frame is ever optimized at one time in this test.

        EXPECT_TRUE(tf_i.isApprox(frame.pose)) << "Optimization result:\n"
                                               << geometry::Log(tf_i) << "\noptimization input:\n"
                                               << geometry::Log(frame.pose);
        EXPECT_TRUE(K.isApprox(intrinsics)) << "Optimization result:\n" << K << "\noptimization input:\n" << intrinsics;
    }
}
