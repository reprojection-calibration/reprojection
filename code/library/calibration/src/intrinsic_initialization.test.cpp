#include "intrinsic_initialization.hpp"

#include <gtest/gtest.h>

#include "eigen_utilities/grid.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

class FocalLengthInitFixture : public ::testing::Test {
   protected:
    // cppcheck-suppress unusedFunction
    void SetUp() override {
        Eigen::ArrayX3d target_points(indices.rows(), 3);
        target_points << indices.cast<double>(), Eigen::ArrayXd::Constant(indices.rows(), 15);
        // NOTE(Jack): We need to get the points off the origin to avoid singularities for the focal length
        // initialization case that would otherwise resul tin no successful gamma estimate below.
        target_points.leftCols<2>().array() -= 3;

        Array5d const ucm_intrinsics{600, 600, width / 2, height / 2, 1};
        auto const camera{projection_functions::UcmCamera(ucm_intrinsics, testing_utilities::image_bounds)};
        auto const [pixels, mask]{camera.Project(target_points)};

        // TODO(Jack): We could also just use the mask to filter out any bad pixels but this also gets the job done, but
        //  it is a brute force method.
        ASSERT_TRUE(mask.all());

        target = ExtractedTarget{{pixels, target_points}, indices};
    }

    ArrayX2i indices{eigen_utilities::GenerateGridIndices(6, 7)};
    ExtractedTarget target;
    double height{480};
    double width{720};
};

// TODO(Jack): Add a helper function/lambda so we do not need to repeat the exact same test logic for each camera type?
// Tested using the double sphere strategy logic but we could have tested it with any of them.
TEST_F(FocalLengthInitFixture, TestSelectInitializationStrategy) {
    // TODO(Jack): Should we just use ucm here to make it more consistent with the test fixture? Right now we only use
    //  double sphere here because it is at this time the only projection class with an Initialization() method? Then we
    //  might be able to check the values exactly like in TestInitializeFocalLengthParabolaLine
    auto [runner, initialization]{calibration::SelectInitializationStrategy(CameraModel::DoubleSphere, height, width)};

    std::vector<double> const ds_gammas{runner(target)};
    EXPECT_EQ(std::size(ds_gammas), 3);  // Simple heuristic to check we get any result from the runner.

    Array6d const ds_intrinsics{initialization(600, height, width)};
    Array6d const gt_ds_intrinsics{300, 300, 360, 240, 0, 0.5};
    EXPECT_TRUE(ds_intrinsics.isApprox(gt_ds_intrinsics));

    // Now we run the other methods too, mainly to get full test coverage, not because we need to.
    std::tie(runner, initialization) =
        calibration::SelectInitializationStrategy(CameraModel::PinholeRadtan4, height, width);
    std::vector<double> const prt4_gammas{runner(target)};
    EXPECT_EQ(std::size(prt4_gammas), 6);  // Simple heuristic to check we get any result from the runner.

    std::tie(runner, initialization) =
        calibration::SelectInitializationStrategy(CameraModel::UnifiedCameraModel, height, width);
    std::vector<double> const ucm_gammas{runner(target)};
    EXPECT_EQ(std::size(ucm_gammas), 3);  // Simple heuristic to check we get any result from the runner.
}

// NOTE(Jack): We use the UCM camera model for testing because the parabola line math is exact when xi=1, which explains
// why we get exactly f=600 below.
TEST_F(FocalLengthInitFixture, TestEstimateCandidatesParabolaLine) {
    auto const result{calibration::EstimateCandidatesParabolaLine(target, width / 2, height / 2)};

    EXPECT_EQ(std::size(result), 4);  // Arbitrary number of successful initializations
    for (auto const f_i : result) {
        EXPECT_FLOAT_EQ(f_i, 600);
    }
}

TEST_F(FocalLengthInitFixture, TestEstimateCandidatesVanishingPoint) {
    auto const result{calibration::EstimateCandidatesVanishingPoint(target)};

    EXPECT_EQ(std::size(result), 6);

    // TODO(Jack): Is there a way to design the test data that we get a single exact value back like above for the
    //  EstimateCandidatesParabolaLine() test, that we can then assert on?
}
