#include <gtest/gtest.h>

#include <string_view>

#include "steps/camera_nonlinear_refinement.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/linear_pose_initialization.hpp"
#include "steps/step_runner.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;
using namespace std::string_view_literals;

class StepsFixture : public ::testing::Test {
   protected:
    // cppcheck-suppress unusedFunction
    void SetUp() override {
        db = std::make_shared<database::CalibrationDatabase>(":memory:", true, false);
        database::WriteToDb(camera_info, db);
    }

    std::shared_ptr<database::CalibrationDatabase> db;
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState camera_state{testing_utilities::pinhole_intrinsics};
};

TEST_F(StepsFixture, TestFeatureExtractionStep) {
    std::vector<std::pair<uint64_t, cv::Mat>> const data{{0, cv::Mat::zeros(10, 20, CV_8UC1)},
                                                         {1, cv::Mat::zeros(10, 20, CV_8UC1)}};
    auto image_source{
        [itr = std::cbegin(data), end = std::cend(data)]() mutable -> std::optional<std::pair<uint64_t, cv::Mat>> {
            if (itr != end) {
                auto const data_i{*itr};
                itr = std::next(itr);
                return data_i;
            }

            return std::nullopt;
        }};

    static constexpr std::string_view config_file{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    application::FeatureExtractionStep const step{image_source, "sha256-key", *config["target"].as_table(),
                                                  *config["sensor"].as_table()};

    // TODO CLEAN UP THIS TYPE!!!!
    auto [result, cache_status]{RunStep<application::FeatureExtractionStep::Result>(step, db)};
    auto& [camera_info, extracted_targets]{result};
    // TODO(Jack): Add helper method to check camera info
    EXPECT_EQ(camera_info.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(camera_info.camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(camera_info.bounds.u_max, 20);
    EXPECT_EQ(camera_info.bounds.v_max, 10);
    EXPECT_EQ(std::size(extracted_targets), 0);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(result, cache_status) = RunStep<application::FeatureExtractionStep::Result>(step, db);
    std::tie(camera_info, extracted_targets) = result;
    EXPECT_EQ(camera_info.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(camera_info.camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(camera_info.bounds.u_max, 20);
    EXPECT_EQ(camera_info.bounds.v_max, 10);
    EXPECT_EQ(std::size(extracted_targets), 0);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(StepsFixture, TestIntrinsicInitializationStep) {
    auto [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 5, 1e9)};
    application::IntrinsicInitializationStep const step{camera_info, targets};

    // NOTE(Jack): Of course it would be best to get the values found in testing_utilities::pinhole_intrinsics as the
    // result, because that is the ground-truth intrinsics. However, the correctness of the pinhole initialization
    // strategy is unclear at this time.
    Array4d const gt_result{287.773, 287.773, 360, 240};  // Heuristic!

    auto [result, cache_status]{RunStep<CameraState>(step, db)};
    EXPECT_TRUE(result.intrinsics.isApprox(gt_result, 1e-3));
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // On rerun with the same inputs it will be a cache hit
    std::tie(result, cache_status) = RunStep<CameraState>(step, db);
    EXPECT_TRUE(result.intrinsics.isApprox(gt_result, 1e-3));
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(StepsFixture, TestLpiStep) {
    auto [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 50, 1e9)};
    application::LpiStep const step{camera_info, targets, camera_state};

    auto [frames, cache_status]{RunStep<Frames>(step, db)};
    EXPECT_EQ(std::size(frames), 50);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // Check that the proper amount of poses got written to the database.
    // TODO(Jack): We should also check that the reprojection errors got written!
    auto poses{database::ReadPoses(db, step.step_type, camera_info.sensor_name)};
    EXPECT_EQ(std::size(poses), 50);

    // On rerun with the same inputs it will be a cache hit
    std::tie(frames, cache_status) = RunStep<Frames>(step, db);
    EXPECT_EQ(std::size(frames), 50);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);

    // Make a new different set of targets to trigger a cache miss and data removal (i.e. sql cascade operation) and
    // replacement with a new set of poses.
    std::tie(targets, gt_poses) = testing_mocks::GenerateMvgData(camera_info, camera_state, 40, 1e9);
    application::LpiStep const step_2{camera_info, targets, camera_state};

    std::tie(frames, cache_status) = RunStep<Frames>(step_2, db);
    EXPECT_EQ(std::size(frames), 40);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    poses = database::ReadPoses(db, step.step_type, camera_info.sensor_name);
    EXPECT_EQ(std::size(poses), 40);
}

TEST_F(StepsFixture, TestCnlrStep) {
    auto [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 50, 1e9)};
    application::CnlrStep const step{camera_info, targets, {camera_state, gt_poses}};

    auto [result, cache_status]{RunStep<OptimizationState>(step, db)};
    EXPECT_EQ(std::size(result.frames), 50);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    auto poses{database::ReadPoses(db, step.step_type, camera_info.sensor_name)};
    EXPECT_EQ(std::size(poses), 50);

    // On rerun with the same inputs it will be a cache hit
    std::tie(result, cache_status) = RunStep<OptimizationState>(step, db);
    EXPECT_EQ(std::size(result.frames), 50);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}