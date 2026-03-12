#include <gtest/gtest.h>

#include "calibration/intrinsic_initialization.hpp"
#include "eigen_utilities/grid.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

class FocalLengthInitFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        Eigen::ArrayXXd target_points(indices.rows(), 3);
        target_points << indices.cast<double>(), Eigen::ArrayXd::Constant(indices.rows(), 15);

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

TEST_F(FocalLengthInitFixture, TestRuntimeInitializationDispatch) {
    // TODO(Jack): Should we just use ucm here to make it more consistent with the test fixture? Right now we only use
    //  double sphere here because it is at this time the only projection class with an Initialization() method? Then we
    //  might be able to check the values exactly like in TestInitializeFocalLengthParabolaLine
    auto const [runner,
                initialization]{calibration::SelectInitializationStrategy(CameraModel::DoubleSphere, height, width)};

    std::vector<double> const gammas{runner(target)};
    EXPECT_EQ(std::size(gammas), 5);  // Simple heuristic to check we get any result from the runner.

    Array6d const ds_intrinsics{initialization(600, height, width)};
    Array6d const gt_ds_intrinsics{300, 300, 360, 240, 0, 0.5};
    EXPECT_TRUE(ds_intrinsics.isApprox(gt_ds_intrinsics));
}

// NOTE(Jack): We use the UCM camera model for testing because the parabola line math is exact when xi=1, which explains
// why we get exactly f=600 below.
TEST_F(FocalLengthInitFixture, TestInitializeFocalLengthParabolaLine) {
    auto const result{calibration::EstimateCandidatesParabolaLine(target, width / 2, height / 2)};

    EXPECT_EQ(std::size(result), 9);  // Arbitrary number of successful initializations
    for (auto const& f_i : result) {
        EXPECT_FLOAT_EQ(f_i, 600);
    }
}
