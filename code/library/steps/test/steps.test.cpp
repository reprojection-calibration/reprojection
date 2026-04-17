#include <gtest/gtest.h>

#include <string_view>

#include "steps/camera_info.hpp"
#include "steps/camera_nonlinear_refinement.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/linear_pose_initialization.hpp"
#include "steps/step_runner.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/io.hpp"

using namespace reprojection;
using namespace std::string_view_literals;

class StepsFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);
        database::WriteToDb(camera_info, db);
    }

    SqlitePtr db;
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState camera_state{testing_utilities::pinhole_intrinsics};
};

// TODO REFACTOR IMAGE SOURCE HERE TO READ DIRECTLR FROM AN ENCODEDIMAGES object!!!
class ImageSourceFixture : public StepsFixture {
   protected:
    void SetUp() override {
        StepsFixture::SetUp();

        // NOTE(Jack): This test kind of captures the data paradigm that we have with our applications. But note that in
        // real applications we create the EncodedImages from the ImageSource. However, in this file we need both of
        // those objects to test the steps here therefore we are clever and use the EncodedImages to construct the
        // ImageSource.

        // Build the encoded images (cv::Mat -> serialized buffer)
        cv::Mat const img{cv::Mat::zeros(10, 20, CV_8UC1)};
        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed");
        }
        encoded_images =
            std::make_shared<EncodedImages>(EncodedImages{{1, ImageBuffer{buffer}}, {2, ImageBuffer{buffer}}});

        // Construct an image source from the encoded images (serialized buffer -> cv::Mat)
        image_source = [itr = std::cbegin(*encoded_images),
                        end = std::cend(*encoded_images)]() mutable -> std::optional<std::pair<uint64_t, cv::Mat>> {
            if (itr != end) {
                auto const& [timestamp_ns, buffer_i]{*itr};
                cv::Mat const img_i{cv::imdecode(buffer_i.data, cv::IMREAD_COLOR)};

                itr = std::next(itr);
                return std::pair{timestamp_ns, img_i};
            }
            return std::nullopt;
        };

        static constexpr std::string_view config_file{R"(
            [sensor]
            camera_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [target]
            pattern_size = [3,4]
            type = "circle_grid"
        )"};
        config = toml::parse(config_file);
    }

    std::shared_ptr<EncodedImages> encoded_images;
    ImageSource image_source;
    toml::table config;
};

TEST_F(ImageSourceFixture, TestImageLoadingStep) {
    steps::ImageLoadingStep const step{camera_info.sensor_name, "sha256-key", image_source};

    auto [encoded_images, cache_status]{RunStep<std::shared_ptr<EncodedImages>>(step, db)};
    EXPECT_EQ(std::size(*encoded_images), 2);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(encoded_images, cache_status) = RunStep<std::shared_ptr<EncodedImages>>(step, db);
    EXPECT_EQ(std::size(*encoded_images), 2);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(ImageSourceFixture, TestCameraInfoStep) {
    steps::CameraInfoStep const step{*config["sensor"].as_table(), encoded_images};

    auto [camera_info, cache_status]{RunStep<CameraInfo>(step, db)};
    EXPECT_EQ(camera_info.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(camera_info.camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(camera_info.bounds.u_max, 20);
    EXPECT_EQ(camera_info.bounds.v_max, 10);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(camera_info, cache_status) = RunStep<CameraInfo>(step, db);
    EXPECT_EQ(camera_info.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(camera_info.camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(camera_info.bounds.u_max, 20);
    EXPECT_EQ(camera_info.bounds.v_max, 10);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(ImageSourceFixture, TestFeatureExtractionStep) {
    steps::FeatureExtractionStep const step{camera_info.sensor_name, "sha256-key", image_source,
                                            *config["target"].as_table()};

    auto [extracted_targets, cache_status]{RunStep<CameraMeasurements>(step, db)};
    EXPECT_EQ(std::size(extracted_targets), 0);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(extracted_targets, cache_status) = RunStep<CameraMeasurements>(step, db);
    EXPECT_EQ(std::size(extracted_targets), 0);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(StepsFixture, TestIntrinsicInitializationStep) {
    auto [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 5, 1e9)};
    steps::IntrinsicInitializationStep const step{camera_info, targets};

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
    steps::LpiStep const step{camera_info, targets, camera_state};

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
    steps::LpiStep const step_2{camera_info, targets, camera_state};

    std::tie(frames, cache_status) = RunStep<Frames>(step_2, db);
    EXPECT_EQ(std::size(frames), 40);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    poses = database::ReadPoses(db, step.step_type, camera_info.sensor_name);
    EXPECT_EQ(std::size(poses), 40);
}

TEST_F(StepsFixture, TestCnlrStep) {
    auto [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 50, 1e9)};
    steps::CnlrStep const step{camera_info, targets, {camera_state, gt_poses}};

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