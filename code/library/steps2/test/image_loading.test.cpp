#include "steps/image_loading.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

// TODO(Jack): Once we refactor the init logic to handle different recordings and config runs we need to test those
// logically too!

TEST(StepsImageLoading, TestManualExecution) {
    // Build the encoded images (cv::Mat -> serialized buffer)
    cv::Mat const img{cv::Mat::zeros(10, 20, CV_8UC1)};
    std::vector<uchar> buffer;
    if (not cv::imencode(".png", img, buffer)) {
        throw std::runtime_error("cv::imencode() failed");
    }
    std::shared_ptr<EncodedImages> const encoded_images{
        std::make_shared<EncodedImages>(EncodedImages{{1, ImageBuffer{buffer}}, {2, ImageBuffer{buffer}}})};

    ImageSampler const image_sampler{
        [itr = std::cbegin(*encoded_images),
         end = std::cend(*encoded_images)]() mutable -> std::optional<std::pair<uint64_t, cv::Mat>> {
            if (itr != end) {
                auto const& [timestamp_ns, buffer_i]{*itr};

                cv::Mat const img_i{cv::imdecode(buffer_i.data, cv::IMREAD_COLOR)};
                itr = std::next(itr);

                return std::pair{timestamp_ns, img_i};
            }
            return std::nullopt;
        }};

    auto db{database::CalibrationDatabase(":memory:", true)};
    AssetId const camera_id{db.GetOrCreateAsset(AssetType::Camera, 0, "")};

    steps::ImageLoading const step{camera_id, "", image_sampler};
    EXPECT_EQ(step.Type(), StepType::ImageLoading);
    // TODO(Jack): We should define a global project constant for the hash of an empty string.
    EXPECT_EQ(step.CacheKey(db).value, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    RecordingId const recording_id{db.GetOrCreateRecording("", "")};
    auto const [step_id, _]{db.GetOrCreateStep(recording_id, std::nullopt, StepType::ImageLoading, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db));

    auto const result{db.ImagesSelect(step_id, camera_id)};
    EXPECT_EQ(std::size(result), 2);
}