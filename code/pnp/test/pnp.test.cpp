#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"

using namespace reprojection::pnp;

TEST(Pnp, TestPnp) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};

        PnpResult const pnp_result{Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(FromSe3(frame_i.pose)));
    }
}

// TODO(Jack): We need to rewrite these tests to check the known theoretical values. For example given certain input
// noise covariances we can predict the results covariancce, or expected residual error etc. This current test simply
// runs it with noise a few times and check that the mean value of all guesses is roughly equal to the input value. This
// is better than nothing but is not exactly scientific...
TEST(Pnp, TestPnpWithNoisyInputData) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    MvgFrame const frame{generator.Generate()};

    int const trials{20};
    Eigen::Matrix<double, Eigen::Dynamic, 6> pose_estimates(trials, 6);
    for (size_t i{0}; i < trials; ++i) {
        // WARN(Jack): Noise is coming from a uniform distribution between -1 and 1, not a normal distribution like you
        // might want.
        Eigen::MatrixX2d const noisy_pixels{frame.pixels + Eigen::MatrixXd::Random(frame.pixels.rows(), 2)};
        Eigen::MatrixX3d const noisy_points{
            frame.points + 0.02 * Eigen::MatrixXd::Random(frame.points.rows(),
                                                          3)};  // Smaller noise scale because we are in translation
                                                                // coordinates here and not pixel coordinates.

        PnpResult const pnp_result{Pnp(noisy_pixels, noisy_points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        pose_estimates.row(i) = ToSe3(pose_i).transpose();
    }

    Se3 const mean_pose_estimate{pose_estimates.colwise().mean()};
    EXPECT_TRUE(mean_pose_estimate.isApprox(frame.pose, 1e-2));  // Heuristic tolerance
}

TEST(Pnp, TestMismatchedCorrespondence) {
    Eigen::MatrixX2d const four_pixels(4, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(four_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::MismatchedCorrespondence);
}

TEST(Pnp, TestNotEnoughPoints) {
    Eigen::MatrixX2d const five_pixels(5, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(five_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::NotEnoughPoints);
}
