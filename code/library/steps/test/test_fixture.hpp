#pragma once

#include <ranges>

#include <testing_utilities/generated/calibration_config.hpp>

#include "database/calibration_database.hpp"
#include "steps/initialize_workflow.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/physics_constants.hpp"

using namespace reprojection;

class StepTestFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        // NOTE(Jack): Here we manually edit the second camera to be a pinhole camera so that way we do not have two
        // cameras with the exact same data! That could interfere with cache hits if the data is the exact same across
        // the two sensors.
        toml::table config{toml::parse(testing_utilities::calibration_config)};
        config["cam1"].as_table()->insert_or_assign("camera_model", "pinhole");

        context_ = steps::InitializeCalibration(config, db_);
    }

    StepId InsertCameraInfo(AssetId const camera_id) {
        auto const camera{GetCamera(camera_id)};
        CameraInfo const camera_info{camera.config.camera_model, testing_utilities::image_bounds};

        StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::CameraInfo, "").first};
        database::CameraInfoInsert(db_.get(), step_id, camera.id, camera_info);

        return step_id;
    }

    StepId InsertTargetInfo() {
        auto const target{context_.assets.target};
        // TODO(Jack): We need a constructor to to this! This is copy and pasted here from the step itself...
        TargetInfo const target_info{target.config.target_type, target.config.size[0], target.config.size[1],
                                     target.config.unit_dimension, target.config.asymmetric};

        StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::TargetInfo, "").first};
        database::TargetInfoInsert(db_.get(), step_id, target.id, target_info);

        return step_id;
    }

    StepId InsertImages(AssetId const camera_id, ImageSamples const& images) {
        auto const step_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
        database::ImagesInsert(db_.get(), step_id, camera_id, images);

        return step_id;
    }

    StepId InsertExtractedTargets(AssetId const camera_id) {
        auto const camera{GetCamera(camera_id)};
        CameraInfo const camera_info{camera.config.camera_model, testing_utilities::image_bounds};

        Intrinsic intrinsic;
        if (camera_info.camera_model == CameraModel::DoubleSphere) {
            intrinsic = {testing_utilities::double_sphere_intrinsics};

        } else if (camera_info.camera_model == CameraModel::Pinhole) {
            intrinsic = {testing_utilities::pinhole_intrinsics};
        } else {
            // LCOV_EXCL_START
            throw std::runtime_error{std::format("Camera model {} not found!", ToString(camera_info.camera_model))};
            // LCOV_EXCL_STOP
        }

        auto const [targets,
                    _]{testing_mocks::GenerateMvgData(camera_info, intrinsic, timing_.duration_s, timing_.camera_hz)};

        // Initialize empty image data using the target timestamps and then write them to the db to satisfy the foreign
        // key constraint.
        ImageSamples const images{[&targets] {
            ImageSamples images;
            for (auto const timestamp_ns : targets | std::views::keys) {
                images.emplace(timestamp_ns, ImageBuffer{});
            }
            return images;
        }()};
        StepId const image_loading_id{InsertImages(camera.id, images)};

        StepId const target_step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};
        database::TargetsInsert(db_.get(), target_step_id, image_loading_id, camera.id, targets);

        return target_step_id;
    }

    StepId InsertIntrinsic(AssetId const camera_id) {
        auto const camera{GetCamera(camera_id)};
        CameraModel const camera_model{camera.config.camera_model};

        // TODO(Jack): This logic is now copy and pasted here and in the extracted target insertion!
        Intrinsic intrinsic;
        if (camera_model == CameraModel::DoubleSphere) {
            intrinsic = {testing_utilities::double_sphere_intrinsics};
        } else if (camera_model == CameraModel::Pinhole) {
            intrinsic = {testing_utilities::pinhole_intrinsics};
        } else {
            // LCOV_EXCL_START
            throw std::runtime_error{std::format("Camera model {} not found!", ToString(camera_model))};
            // LCOV_EXCL_STOP
        }

        StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::BundleAdjustment, "").first};
        database::IntrinsicInsert(db_.get(), step_id, camera_id, camera_model, intrinsic);

        return step_id;
    }

    StepId InsertPoses(AssetId const camera_id, StepId const target_step_id) {
        auto const camera{GetCamera(camera_id)};
        CameraInfo const camera_info{camera.config.camera_model, testing_utilities::image_bounds};

        // TODO(Jack): Logic is now repeated three times!
        Intrinsic intrinsic;
        if (camera_info.camera_model == CameraModel::DoubleSphere) {
            intrinsic = {testing_utilities::double_sphere_intrinsics};
        } else if (camera_info.camera_model == CameraModel::Pinhole) {
            intrinsic = {testing_utilities::pinhole_intrinsics};
        } else {
            // LCOV_EXCL_START
            throw std::runtime_error{std::format("Camera model {} not found!", ToString(camera_info.camera_model))};
            // LCOV_EXCL_STOP
        }

        auto const [_, poses]{
            testing_mocks::GenerateMvgData(camera_info, intrinsic, timing_.duration_s, timing_.camera_hz)};

        StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::BundleAdjustment, "").first};
        database::CameraPosesInsert(db_.get(), step_id, target_step_id, camera.id, poses);

        return step_id;
    }

    StepId InsertExtrinsic(AssetId const asset_a, AssetId const asset_b) {
        StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ExtrinsicInit, "").first};

        // WARN(Jack): Technically this identity transform is actually internal state of the core data generation code.
        // We should be getting these values from the data generation code directly and not hardcode them here.
        Extrinsic const extrinsic{asset_a, asset_b, Array6d::Zero()};
        database::ExtrinsicInsert(db_.get(), step_id, extrinsic);

        return step_id;
    }

    // TODO(Jack): Does both the imu data itself and source spline! If it really makes sense to do both at once is still
    // not clear. But for now at least it keeps the problem setup smaller.
    // TODO(Jack): This camera_id here is not just any random one, it is also the camera which defines the rig/is the
    // reference camera. Should we reflect in the naming or any other way?
    std::pair<StepId, StepId> InsertImuSetup(AssetId const camera_id) {
        auto const imu_id{context_.assets.imu};
        if (not imu_id) {
            throw std::runtime_error{std::format("Imu asset not found!")};  // LCOV_EXCL_LINE
        }

        // WARN(Jack): Normally the spline would be initialized with the poses from the camera initialization. Here
        // however we get the poses and interpolated spline directly from the data generator.
        auto const [imu_data, spline]{testing_mocks::GenerateImuData(timing_.duration_s, timing_.imu_hz)};

        StepId const imu_data_id{database::GetOrCreateStep(db_.get(), StepType::ImuDataLoading, "").first};
        database::ImuDataInsert(db_.get(), imu_data_id, imu_id->id, imu_data);

        StepId const spline_id{database::GetOrCreateStep(db_.get(), StepType::SplineInit, "").first};
        database::ControlPointsInsert(db_.get(), spline_id, camera_id, spline.ControlPoints());
        database::SplineInfoInsert(db_.get(), spline_id, camera_id, spline.GetTimeHandler());

        return {imu_data_id, spline_id};
    }

    StepId InsertImuCamExtrinsic(AssetId const imu_id, AssetId const camera_id) {
        StepId const step_id{InsertExtrinsic(imu_id, camera_id)};

        // WARN(Jack): Technically this gravity value is actually internal state of the core data generation code. We
        // should be getting these values from the data generation code directly and not hardcode them here.
        Array3d const gravity{Array3d{0, 0, kGravity}};
        database::GravityInsert(db_.get(), step_id, gravity);

        return step_id;
    }

    static ImageSamples TestImageSamples() {
        cv::Mat const img{cv::Mat::zeros(10, 20, CV_8UC1)};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed");  // LCOV_EXCL_LINE
        }
        ImageSamples const encoded_images{{{1, ImageBuffer{buffer}}}};

        return encoded_images;
    }

   private:
    Asset<config::Config::Camera> GetCamera(AssetId const camera_id) {
        auto const& cameras{context_.assets.cameras};
        auto const it{std::ranges::find_if(context_.assets.cameras,
                                           [camera_id](auto const& camera) { return camera.id == camera_id; })};
        if (it == std::cend(cameras)) {
            throw std::runtime_error{std::format("Camera asset id {} not found!", camera_id.value)};  // LCOV_EXCL_LINE
        }

        return *it;
    }

   public:
    SqlitePtr db_{database::OpenCalibrationDatabase(":memory:", true)};
    steps::CalibrationContext context_;

    // NOTE(Jack): At least one test (extrinsic optimization) requires higher frequency data to return a correct result
    // so we make this parameterizable but set default values which are valid for all other tests.
    struct TimingParameters {
        double duration_s{11};
        double camera_hz{1};
        double imu_hz{5};
    };

    TimingParameters timing_{};
};
