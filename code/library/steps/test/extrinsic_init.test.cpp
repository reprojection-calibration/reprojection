#include "steps/extrinsic_init.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "types/physics_constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class ExtrinsicInitFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        database::AssetGroupInsert(db_.get(), {camera_id_, imu_id_});

        auto const [imu_data, spline]{testing_mocks::GenerateImuData(11, 5)};

        database::ImuDataInsert(db_.get(), imu_data_id_, imu_id_, imu_data);
        database::ControlPointsInsert(db_.get(), spline_id_, camera_id_, spline.ControlPoints());
        database::SplineInfoInsert(db_.get(), spline_id_, camera_id_, spline.GetTimeHandler());
    }

    SqlitePtr db_{database::OpenCalibrationDatabase(":memory:", true)};
    AssetId camera_id_{database::GetOrCreateAsset(db_.get(), AssetType::Camera, 0, "")};
    AssetId imu_id_{database::GetOrCreateAsset(db_.get(), AssetType::Imu, 0, "")};
    StepId imu_data_id_{database::GetOrCreateStep(db_.get(), StepType::ImuDataLoading, "").first};
    StepId spline_id_{database::GetOrCreateStep(db_.get(), StepType::SplineInit, "").first};
};

TEST_F(ExtrinsicInitFixture, TestExtrinsicInitStepRunner) {
    WorkflowId const workflow_id{database::GetOrCreateWorkflow(db_.get(), WorkflowType::CamImu, {camera_id_, imu_id_})};
    steps::ExtrinsicInit const step{camera_id_, spline_id_, imu_id_, imu_data_id_, 1, db_};
    StepId const step_id{RunStep<steps::ExtrinsicInit>(workflow_id, step, db_)};

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);  // Heuristic!

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);  // Heuristic!
}

TEST_F(ExtrinsicInitFixture, TestExtrinsicInitStep) {
    steps::ExtrinsicInit const step{camera_id_, spline_id_, imu_id_, imu_data_id_, 1, db_};
    EXPECT_EQ(step.Type(), StepType::ExtrinsicInit);
    std::vector const gt_assets{camera_id_, imu_id_};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "d78f7d0b3bf9ef156ed4b8c9c31eaf1fcefb3174b239d1b5e471de80c488bc05");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ExtrinsicInit, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);  // Heuristic!

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);  // Heuristic!
}