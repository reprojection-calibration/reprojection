#include "optimization/bundle_adjustment.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "geometry/lie.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

using Ba = optimization::BundleAdjustment;

class BaFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        std::tie(targets_, frames_) = testing_mocks::GenerateMvgData(camera_info_, intrinsic_, 60, 1, false);

        ASSERT_EQ(std::size(targets_), 56);
        ASSERT_EQ(std::size(targets_), std::size(frames_));
    }

    AssetId camera_id_{1};  // cppcheck-suppress unusedStructMember
    CameraInfo camera_info_{CameraModel::Pinhole, testing_utilities::image_bounds};
    Intrinsic intrinsic_{testing_utilities::pinhole_intrinsics};
    TargetSamples targets_;
    Frames frames_;
};

TEST_F(BaFixture, TestMultiCam) {
    // NOTE(Jack): The real meat and potatoes of this test is that we initialize the second and third cam with random
    // non zero extrinsic. We then assert that we recover the identity extrinsic after the optimization.
    std::vector<optimization::CameraProblemInput> const cams{
        {camera_id_, camera_info_, intrinsic_, targets_, Array6d::Zero(), true, false},
        {AssetId{2}, camera_info_, intrinsic_, targets_, Array6d::Random(), true, true},
        {AssetId{3}, camera_info_, intrinsic_, targets_, Array6d::Random(), true, true},
    };

    Ba::Problem const problem{Ba::MultiCamProblem(camera_id_, frames_, cams, 0)};

    auto const [result, ceres_state]{Ba::Solve(problem, 1)};
    EXPECT_EQ(ceres_state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    auto const& [ref_asset, frames, cameras]{result};
    EXPECT_EQ(ref_asset, camera_id_);
    EXPECT_EQ(std::size(cameras), 3);

    // Assert
    EXPECT_EQ(std::size(frames), 56);
    for (auto const& [timestamp_ns, frame_i] : frames) {
        Array6d const gt_se3_co_w{frames_.at(timestamp_ns).value};
        Array6d const se3_co_w{frame_i.value};

        EXPECT_TRUE(se3_co_w.isApprox(gt_se3_co_w, 1e-6)) << "Result:\n"
                                                          << se3_co_w.transpose() << "\nexpected result:\n"
                                                          << gt_se3_co_w.transpose();
    }

    for (auto const& camera_i : cameras | std::views::values) {
        auto const intrinsic_i{camera_i.intrinsic.value};
        EXPECT_TRUE(intrinsic_i.isApprox(intrinsic_.value, 1e-6)) << "Result:\n"
                                                                  << intrinsic_i.transpose() << "\nexpected result:\n"
                                                                  << intrinsic_.value.transpose();

        // TODO(Jack): It would be smarter to get the ground truth extrinsic from some sort of problem creation logic,
        // but for now we just hardcode it to identity like we have done elsewhere. Not clean!
        auto const extrinsic_i{camera_i.extrinsic};
        EXPECT_TRUE(extrinsic_i.isZero(1e-9));
    }
}

// Test with perfect data - means inputs will be exact same as outputs. Technically this test might miss something
// because the optimization will likely not even execute once because the error is zero. For a real test look at the
// next case where we add some noisy so it actually does some iterations.
TEST_F(BaFixture, TestBundleAdjustmentBatch) {
    Ba::Problem const problem{Ba::SingleCamProblem(camera_info_, intrinsic_, targets_, frames_, true, camera_id_)};

    auto const [result, ceres_state]{Ba::Solve(problem, 1)};
    EXPECT_EQ(ceres_state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    auto const& [ref_asset, frames, cameras]{result};
    EXPECT_EQ(ref_asset, camera_id_);

    // Assert
    EXPECT_EQ(std::size(frames), 56);
    for (auto const& [timestamp_ns, frame_i] : frames) {
        Array6d const gt_se3_co_w{frames_.at(timestamp_ns).value};
        Array6d const se3_co_w{frame_i.value};

        EXPECT_TRUE(se3_co_w.isApprox(gt_se3_co_w, 1e-6)) << "Result:\n"
                                                          << se3_co_w.transpose() << "\nexpected result:\n"
                                                          << gt_se3_co_w.transpose();
    }

    auto const intrinsic{cameras.at(camera_id_).intrinsic.value};
    EXPECT_TRUE(intrinsic.isApprox(intrinsic_.value, 1e-6)) << "Result:\n"
                                                            << intrinsic.transpose() << "\nexpected result:\n"
                                                            << intrinsic_.value.transpose();
}

// Given a noisy initial pose but perfect bundle (i.e. no noise in the pixels or points), we then get perfect poses
// and intrinsic back.
TEST_F(BaFixture, TestNoisyBundleAdjustment) {
    // Add gaussian noise to the initial poses
    Frames noisy_frames{frames_};
    for (auto& [_, frame_i] : noisy_frames) {
        Isometry3d const SE3_i{geometry::Exp(frame_i.value)};
        frame_i.value = geometry::Log(testing_mocks::AddGaussianNoise(0.1, 0.1, SE3_i));
    }

    Ba::Problem const problem{Ba::SingleCamProblem(camera_info_, intrinsic_, targets_, noisy_frames, true, camera_id_)};

    auto const [result, ceres_state]{Ba::Solve(problem, 1)};
    EXPECT_EQ(ceres_state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    auto const& [ref_asset, frames, cameras]{result};
    EXPECT_EQ(ref_asset, camera_id_);

    EXPECT_EQ(std::size(frames), 56);
    for (auto const& [timestamp_ns, frame_i] : frames) {
        // WARN(Jack): Clearly I do not understand the axis-angle representation... And here something frustrating
        // happened that I will explain. This test using noisy poses had been working for months, no problems to report.
        // Comparing the Vector6d se3 poses directly worked perfectly and the optimization returned the ground truth
        // value. Then suddenly, when we transitioned to the "view" concept, this test started to fail. And what would
        // happen is that the optimized answer would have he right translation but the se3 axis-angle rotation would be
        // flipped (similar to what we observed when plotting the poses in Dash). It was not flipped for all poses, but
        // only sometimes and seemingly randomly. As I still do not have a solution for this, we actually changed this
        // test to instead compare the 4x4 SE3 transformation  matrices. Now it passes again, essentially the same as
        // before, but now working in the matrix space. Why all of a sudden the optimized poses start flipping, I cannot
        // explain.
        Isometry3d const gt_tf_co_w{geometry::Exp(frames_.at(timestamp_ns).value)};
        Isometry3d const tf_co_w{geometry::Exp(frame_i.value)};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w, 1e-6)) << "Result:\n"
                                                        << tf_co_w.matrix() << "\nexpected result:\n"
                                                        << gt_tf_co_w.matrix();
    }

    auto const intrinsic{cameras.at(camera_id_).intrinsic.value};
    EXPECT_TRUE(intrinsic.isApprox(intrinsic_.value, 1e-6)) << "Result:\n"
                                                            << intrinsic.transpose() << "\nexpected result:\n"
                                                            << intrinsic_.value.transpose();
}

TEST_F(BaFixture, TestToRigState) {
    std::map<AssetId, Ba::CameraState> camera_states{{camera_id_, {intrinsic_, Array6d::Zero()}}};
    Ba::Result data{camera_id_, frames_, camera_states};

    // Single camera case - a special case where the rig_frame_asset_id is the came as the only camera state present.
    auto result{optimization::ToRigState(data)};
    EXPECT_EQ(result.rig_frame_asset_id, camera_id_);
    EXPECT_EQ(std::size(result.poses), 56);
    // NOTE(Jack): At time of writing (08.09.2026) the identity self extrinsic is not supported because cycles are not
    // supported by out Extrinsics{} abstraction. Therefore the rig state here has no extrinsic values stored, because
    // if you query the ref_id->ref_id extrinsic it automatically knows that is the identity and returns it.
    EXPECT_EQ(std::size(result.extrinsics.Values()), 0);

    // Add a second non-reference camera - no we actually will have an extrinsic transform in the extrinsics state.
    AssetId const second_cam{2};
    data.camera_states.insert({second_cam, {{Array3d::Zero()}, Array6d::Zero()}});

    result = optimization::ToRigState(data);
    EXPECT_EQ(result.rig_frame_asset_id, camera_id_);
    EXPECT_EQ(std::size(result.poses), 56);
    EXPECT_EQ(std::size(result.extrinsics.Values()), 1);
    EXPECT_TRUE(result.extrinsics.HasPath(camera_id_, second_cam));
}

TEST_F(BaFixture, TestReprojectionError) {
    // NOTE(Jack): The real ground truth value for both the valid pixels here is actually the center of the image (i.e.
    // [360, 240])! But because we want to see that the reprojection error is actually the correct value we make the
    // "ground truth" pixels here have some error.
    MatrixX2d const gt_pixels{{-1, -1},  //
                              {350, 230},
                              {-1, -1},
                              {-1, -1},
                              {365, 245}};
    MatrixX3d const gt_points{{0, 0, -600},  //
                              {0, 0, 600},
                              {0, 0, -600},
                              {0, 0, -600},
                              {0, 0, 600}};
    // If the pixel evaluation fails then the cost function will automatically fill out the residual value with 256,
    // this is arbitrary and heuristic. See the note in the projection cost function implementation.
    ArrayX2d const gt_residuals{{256, 256},  //
                                {-10, -10},
                                {256, 256},
                                {256, 256},
                                {5, 5}};

    uint64_t const timestamp_ns{0};  // Used to track the data frame in the maps
    Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
    TargetSamples const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};

    Ba::Problem const problem{Ba::SingleCamProblem(camera_info_, intrinsic_, targets, frames, false, camera_id_)};

    auto const residuals{optimization::EvaluateResiduals(problem)};
    EXPECT_EQ(std::size(residuals), 1);

    // There is only one value so we hardcode index into the 0 spot. Does not scale but works for the test!
    auto const& residual{residuals[0]};
    EXPECT_EQ(residual.camera_id, camera_id_);
    EXPECT_EQ(residual.timestamp_ns, timestamp_ns);
    EXPECT_TRUE(residual.value.isApprox(gt_residuals)) << "Result:\n"
                                                       << residual.value.transpose() << "\nexpected result:\n"
                                                       << gt_residuals.transpose();
}