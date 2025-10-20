#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;
using namespace reprojection::pnp;

TEST(Pnp, TestPnp) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(false)};
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        PnpResult const pnp_result{Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose));
    }
}

// TODO(Jack): Add noisy point test for Dlt22 path of Pnp
TEST(Pnp, TestPnpFlat) {
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // Pixels must be in normalized space for Dlt22
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(true, K)};  // Points must have Z=0 (flat = true)

    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        PnpResult const pnp_result{Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose));
    }
}

// TODO(Jack): We need to rewrite these tests to check the known theoretical values. For example given certain input
// noise covariances we can predict the results covariance, or expected residual error etc. This current test simply
// runs it with noise a few times and check that the mean value of all guesses is roughly equal to the input value. This
// is better than nothing but is not exactly scientific...
TEST(Pnp, TestPnpWithNoisyInputData) {
    // NOTE(Jack): We need a really long focal length here so that the features cover a large part of the image to make
    // the noise have less of an impact.
    Eigen::Matrix3d const K{{1800, 0, 360}, {0, 1800, 240}, {0, 0, 1}};  // Pixels must be in normalized space for Dlt22
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(false, K)};
    testing_mocks::MvgFrame const frame{generator.Generate(0.5)};  // Arbitrary spot in the middle

    int const trials{20};
    Eigen::Matrix<double, Eigen::Dynamic, 6> pose_estimates(trials, 6);
    for (size_t i{0}; i < trials; ++i) {
        // WARN(Jack): Noise is coming from a uniform distribution between -1 and 1, not a normal distribution like you
        // might want.
        Eigen::MatrixX2d const noisy_pixels{frame.pixels + Eigen::MatrixXd::Random(frame.pixels.rows(), 2)};
        // WARN(Jack): This is a really really tiny noise multiplier! Maybe 25 points is too low to be robust to noise
        // in the points.
        Eigen::MatrixX3d const noisy_points{
            frame.points + 0.0002 * Eigen::MatrixXd::Random(frame.points.rows(),
                                                            3)};  // Smaller noise scale because we are in translation
                                                                  // coordinates here and not pixel coordinates.

        PnpResult const pnp_result{Pnp(noisy_pixels, noisy_points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        pose_estimates.row(i) = geometry::Log(pose_i).transpose();
    }

    Eigen::Vector<double, 6> const mean_pose_estimate{pose_estimates.colwise().mean()};
    EXPECT_TRUE(mean_pose_estimate.isApprox(geometry::Log(frame.pose), 1e-2));  // Heuristic tolerance
}

TEST(Pnp, TestMismatchedCorrespondence) {
    Eigen::MatrixX2d const four_pixels(4, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(four_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::MismatchedCorrespondences);
}

TEST(Pnp, TestNotEnoughPoints) {
    Eigen::MatrixX2d const five_pixels(5, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(five_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::NotEnoughPoints);
}
