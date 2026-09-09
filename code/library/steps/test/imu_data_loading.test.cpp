#include "steps/imu_data_loading.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class ImuSamplerFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        imu_id_ = context_.assets.imu->id;  // Unprotected optional access!
        imu_data_ = ImuSamples{{0, {{1, 2, 3}, {4, 5, 6}}}, {1, {{1, 2, 3}, {4, 5, 6}}}};
        imu_sampler_ =
            [itr = std::cbegin(imu_data_),
             end = std::cend(imu_data_)]() mutable -> std::optional<std::pair<uint64_t, std::array<double, 6>>> {
            if (itr != end) {
                auto const& [timestamp_ns, imu_data_i]{*itr};
                std::array const imu_data_i_array{
                    imu_data_i.angular_velocity(0),    imu_data_i.angular_velocity(1),
                    imu_data_i.angular_velocity(2),    imu_data_i.linear_acceleration(0),
                    imu_data_i.linear_acceleration(1), imu_data_i.linear_acceleration(2),
                };

                itr = std::next(itr);

                return std::pair{timestamp_ns, imu_data_i_array};
            }
            return std::nullopt;
        };
    }

    AssetId imu_id_;
    ImuSamples imu_data_;
    ImuSampler imu_sampler_;
};

TEST_F(ImuSamplerFixture, TestImuDataLoadingStepRunner) {
    steps::ImuDataLoading const step{imu_id_, "", imu_sampler_};
    StepId const step_id{RunStep<steps::ImuDataLoading>(context_.workflow_id, step, db_)};

    auto const result{database::ImuDataSelect(db_.get(), step_id, imu_id_)};
    EXPECT_EQ(std::size(result), std::size(imu_data_));

    for (auto const timestamp_ns : imu_data_ | std::views::keys) {
        auto const& result_i{result.at(timestamp_ns)};
        auto const& input_i{imu_data_.at(timestamp_ns)};

        EXPECT_TRUE(result_i.angular_velocity.isApprox(input_i.angular_velocity));
        EXPECT_TRUE(result_i.linear_acceleration.isApprox(input_i.linear_acceleration));
    }
}

TEST_F(ImuSamplerFixture, TestImuDataLoadingStep) {
    // Build the step and check that the basics function are correct.
    steps::ImuDataLoading const step{imu_id_, "", imu_sampler_};

    EXPECT_EQ(step.Type(), StepType::ImuDataLoading);
    EXPECT_EQ(step.Assets(), std::vector{imu_id_});
    EXPECT_EQ(step.CacheKey().value, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    // Build the actual database step id and execute the step.
    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::ImuDataLoading, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    // Load the result and check it's the same as the input
    auto const result{database::ImuDataSelect(db_.get(), step_id, imu_id_)};
    EXPECT_EQ(std::size(result), std::size(imu_data_));

    for (auto const timestamp_ns : imu_data_ | std::views::keys) {
        auto const& result_i{result.at(timestamp_ns)};
        auto const& input_i{imu_data_.at(timestamp_ns)};

        EXPECT_TRUE(result_i.angular_velocity.isApprox(input_i.angular_velocity));
        EXPECT_TRUE(result_i.linear_acceleration.isApprox(input_i.linear_acceleration));
    }
}