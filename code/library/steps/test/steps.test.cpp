#include <gtest/gtest.h>

#include <ranges>
#include <string_view>

#include "config/config_parsing.hpp"
#include "steps/bundle_adjustment.hpp"
#include "steps/camera_info.hpp"
#include "steps/extrinsic_initialization.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/image_loading.hpp"
#include "steps/imu_data_loading.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/pose_initialization.hpp"
#include "steps/spline_initialization.hpp"
#include "steps/step_runner.hpp"
#include "steps/target_info.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

// cppcheck-suppress missingInclude
#include "testing_utilities/generated/minimum_config.hpp"
#include "types/io.hpp"

using namespace reprojection;
using namespace std::string_view_literals;

// TODO(Jack): Rename to reflect this is specific to a camera entity.
class CameraStepsFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        config = toml::parse(testing_utilities::minimum_config);

        auto const [sensor_name, camera_model]{config::ParseCameraConfig(*config["camera"].as_table())};
        camera_info = CameraInfo{sensor_name, camera_model, testing_utilities::image_bounds};

        // WARN(Jack): Make sure all tests that use this are really a camera!!!
        database::InsertEntity(db, camera_info.sensor_name, Entity::Camera);
        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::CameraInfo, "");
        database::InsertCameraInfo(db, camera_info);
    }

    void SatisfyPoseForeignKeys(CameraMeasurements const& targets) {
        // Satisfy the foreign key constraints that the BundleAdjustment/PoseInitialization steps have (i.e. we need to
        // be able to write poses). This means writing the images which we construct here in this lambda and also the
        // extracted targets which the mvg data generator provides us directly.
        EncodedImages const images{[&targets]() {
            EncodedImages images;
            for (auto const timestamp_ns : targets | std::ranges::views::keys) {
                images.insert({timestamp_ns, {}});
            }
            return images;
        }()};

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::ImageLoading, "");
        database::InsertImages(db, camera_info.sensor_name, images);
        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::FeatureExtraction, "");
        database::InsertTargets(db, camera_info.sensor_name, targets);
    }

    SqlitePtr db;
    CameraInfo camera_info;
    CameraState camera_state{testing_utilities::double_sphere_intrinsics};
    toml::table config;
};

class ImageSourceFixture : public CameraStepsFixture {
   protected:
    void SetUp() override {
        CameraStepsFixture::SetUp();

        // NOTE(Jack): This test kind of captures the data paradigm that we have with our applications. But note that in
        // real applications we create the EncodedImages from the ImageSourceSignature. However, in this file we need
        // both of those objects to test the steps here therefore we are clever and use the EncodedImages to construct
        // the ImageSourceSignature.

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

        // TODO(Jack): This conversion logic is now at least repeated here and in the target info step exactly the same,
        // this could be good place for a reusable config parsing function instead of copy and paste.
        target_info = config::ParseTargetConfig(*config["target"].as_table());
    }

    std::shared_ptr<EncodedImages> encoded_images;
    ImageSourceSignature image_source;
    TargetInfo target_info;
};

TEST_F(ImageSourceFixture, TestImageLoading) {
    steps::ImageLoading const step{camera_info.sensor_name, "sha256-key", image_source};

    auto [encoded_images, cache_status]{RunStep<std::shared_ptr<EncodedImages>>(step, db)};
    EXPECT_EQ(std::size(*encoded_images), 2);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(encoded_images, cache_status) = RunStep<std::shared_ptr<EncodedImages>>(step, db);
    EXPECT_EQ(std::size(*encoded_images), 2);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(ImageSourceFixture, TestCameraInfoStep) {
    steps::CameraInfoStep const step{*config["camera"].as_table(), encoded_images};

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

TEST_F(ImageSourceFixture, TestFeatureExtraction) {
    // NOTE(Jack): There are no foreign key constraints that need to be satisfied here because there are no actuall
    // extracted targets which get written to or loaded from the database.

    steps::FeatureExtraction const step{camera_info.sensor_name, encoded_images, target_info, false};

    auto [extracted_targets, cache_status]{RunStep<CameraMeasurements>(step, db)};
    EXPECT_EQ(std::size(extracted_targets), 0);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(extracted_targets, cache_status) = RunStep<CameraMeasurements>(step, db);
    EXPECT_EQ(std::size(extracted_targets), 0);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(CameraStepsFixture, TestBundleAdjustmentStep) {
    auto const [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 60, 1)};

    SatisfyPoseForeignKeys(targets);

    steps::BundleAdjustment const step{camera_info, targets, {camera_state, gt_poses}};

    auto [result, cache_status]{RunStep<OptimizationState>(step, db)};
    EXPECT_EQ(std::size(result.frames), 56);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    auto const poses{database::ReadPoses(db, camera_info.sensor_name, step.StepType())};
    EXPECT_EQ(std::size(poses), 56);

    // On rerun with the same inputs it will be a cache hit
    std::tie(result, cache_status) = RunStep<OptimizationState>(step, db);
    EXPECT_EQ(std::size(result.frames), 56);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(CameraStepsFixture, TestIntrinsicInitialization) {
    auto const [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 60, 1)};
    steps::IntrinsicInitialization const step{camera_info, targets};

    // NOTE(Jack): Of course it would be best to get the values found in testing_utilities::pinhole_intrinsics as the
    // result, because that is the ground-truth intrinsics. However, the correctness of the pinhole initialization
    // strategy is unclear at this time.
    Array5d const gt_result{535.023, 360, 240, 0, 0.5};  // Heuristic!

    auto [result, cache_status]{RunStep<CameraState>(step, db)};
    EXPECT_TRUE(result.intrinsics.isApprox(gt_result, 1e-3));
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // On rerun with the same inputs it will be a cache hit
    std::tie(result, cache_status) = RunStep<CameraState>(step, db);
    EXPECT_TRUE(result.intrinsics.isApprox(gt_result, 1e-3));
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(CameraStepsFixture, TestPoseInitialization) {
    auto [targets, gt_poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 60, 1)};

    SatisfyPoseForeignKeys(targets);

    steps::PoseInitialization const step{camera_info, targets, camera_state};

    auto [frames, cache_status]{RunStep<Frames>(step, db)};
    EXPECT_EQ(std::size(frames), 56);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // Check that the proper amount of poses got written to the database.
    // TODO(Jack): We should also check that the reprojection errors got written!
    auto poses{database::ReadPoses(db, camera_info.sensor_name, step.StepType())};
    EXPECT_EQ(std::size(poses), 56);

    // On rerun with the same inputs it will be a cache hit
    std::tie(frames, cache_status) = RunStep<Frames>(step, db);
    EXPECT_EQ(std::size(frames), 56);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(CameraStepsFixture, TestSplineInitialization) {
    auto const [targets, poses]{testing_mocks::GenerateMvgData(camera_info, camera_state, 10, 1)};

    SatisfyPoseForeignKeys(targets);

    steps::SplineInitialization const step{camera_info, targets, {camera_state, poses}};

    auto [result, cache_status]{RunStep<spline::Se3Spline>(step, db)};
    EXPECT_EQ(result.Size(), 558);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    auto const control_points{database::ReadControlPoints(db, camera_info.sensor_name, step.StepType())};
    EXPECT_EQ(control_points.cols(), 558);

    // On rerun with the same inputs it will be a cache hit
    std::tie(result, cache_status) = RunStep<spline::Se3Spline>(step, db);
    EXPECT_EQ(result.Size(), 558);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST_F(CameraStepsFixture, TestTargetInfoStep) {
    steps::TargetInfoStep const step{*config["target"].as_table(), camera_info.sensor_name};

    auto [target_info, cache_status]{RunStep<TargetInfo>(step, db)};
    EXPECT_EQ(target_info.target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(target_info.height, 3);
    EXPECT_EQ(target_info.width, 4);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(target_info, cache_status) = RunStep<TargetInfo>(step, db);
    EXPECT_EQ(target_info.target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(target_info.height, 3);
    EXPECT_EQ(target_info.width, 4);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST(StepsSteps, TestExtrinsicInitialization) {
    SqlitePtr db{database::OpenCalibrationDatabase(":memory:", true, false)};

    std::string const imu_name{"/imu/polaris/123"};
    database::InsertEntity(db, imu_name, Entity::Imu);
    std::string const camera_name{"/cam0/image_raw"};
    database::InsertEntity(db, camera_name, Entity::Camera);

    // TODO(Jack): This extrinsic entity id logic is copy and pasted from the step, is there a better way to unify
    // this and make the extrinsic entity id a first class concept?
    std::string const extrinsic_id{Extrinsic::EntityId(imu_name, camera_name)};
    database::InsertEntity(db, extrinsic_id, Entity::Extrinsic);

    // NOTE(Jack): Normally the extrinsic initialization function will actually run against the camera frames which I
    // think are inverted compared to the spline returned by the imu data generation function here. The proper way to
    // get the camera orientation spline would actually be to also run the mvg data generation and then interpolate the
    // frames returned from there. But this test does not need algorithmic correctness as we are just wanting to test
    // the process mechanics. But still it would be nice to get ,a "proper" result here so maybe we change this.
    auto const [imu_data, spline_b_w]{testing_mocks::GenerateImuData(20, 50)};

    // Satisfy foreign key constraint because the Save() stage will write out ImuErrors which depend on having a
    // correspondent IMU data point.
    database::InsertStep(db, imu_name, CalibrationStep::ImuDataLoading, "");
    database::InsertImuData(db, imu_name, imu_data);

    steps::ExtrinsicInitialization const step{imu_name, camera_name, imu_data, spline_b_w};
    auto [result, cache_status]{RunStep<ImuCamExtrinsic>(step, db)};

    Array3d const gravity_w_gt{-5.4104203543938996e-05, 0.019931736220415951, 9.8066297444873474};

    EXPECT_EQ(result.tf.frame_a, imu_name);
    EXPECT_EQ(result.tf.frame_b, camera_name);
    EXPECT_TRUE(result.tf.se3_a_b.isZero(1e-4));
    EXPECT_TRUE(result.gravity.isApprox(gravity_w_gt));
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    // On rerun with the same inputs it will be a cache hit
    std::tie(result, cache_status) = RunStep<ImuCamExtrinsic>(step, db);
    EXPECT_EQ(result.tf.frame_a, imu_name);
    EXPECT_EQ(result.tf.frame_b, camera_name);
    EXPECT_TRUE(result.tf.se3_a_b.isZero(1e-4));
    EXPECT_TRUE(result.gravity.isApprox(gravity_w_gt));
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}

TEST(StepsSteps, TestImuDataLoading) {
    SqlitePtr db{database::OpenCalibrationDatabase(":memory:", true, false)};
    std::string const imu_name{"imu"};

    // TODO(Jack): As we add more tests this should probable be packed into a test fixture.
    ImuMeasurements const gt_imu_data{{0, {Array3d::Ones(), Array3d::Ones()}}, {1, {Array3d::Ones(), Array3d::Ones()}}};
    ImuDataSourceSignature imu_data_source{
        [itr = std::cbegin(gt_imu_data),
         end = std::cend(gt_imu_data)]() mutable -> std::optional<std::pair<uint64_t, std::array<double, 6>>> {
            if (itr != end) {
                auto const& [timestamp_ns, data_i]{*itr};

                std::array<double, 6> const array{data_i.angular_velocity[0],    data_i.angular_velocity[1],
                                                  data_i.angular_velocity[2],    data_i.linear_acceleration[0],
                                                  data_i.linear_acceleration[1], data_i.linear_acceleration[2]};

                itr = std::next(itr);
                return std::pair{timestamp_ns, array};
            }
            return std::nullopt;
        }};

    // Satisfy foreign key constraints
    database::InsertEntity(db, imu_name, Entity::Imu);

    steps::ImuDataLoading const step{imu_name, "", imu_data_source};

    auto [imu_data, cache_status]{RunStep<ImuMeasurements>(step, db)};
    EXPECT_EQ(std::size(imu_data), 2);
    EXPECT_EQ(cache_status, CacheStatus::CacheMiss);

    std::tie(imu_data, cache_status) = RunStep<ImuMeasurements>(step, db);
    EXPECT_EQ(std::size(imu_data), 2);
    EXPECT_EQ(cache_status, CacheStatus::CacheHit);
}