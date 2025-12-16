#include "database/image_interface.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>

#include "database/calibration_database.hpp"

using namespace reprojection;

// Test fixture used to facilitate isolated filesystem state. This is useful when testing database creation to prevent
// artefacts from previous or parallel testing interfering with the system currently under test.
class TempFolder : public ::testing::Test {
   protected:
    void SetUp() override { std::filesystem::create_directories(database_path_); }

    void TearDown() override { std::filesystem::remove_all(database_path_); }

    std::string database_path_{"sandbox"};
};

TEST_F(TempFolder, TestAddImage) {
    std::string const record_path{database_path_ + "/record_uuu.db3"};
    cv::Mat const image(480, 720, CV_8UC1);
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    EXPECT_TRUE(database::AddImage("/cam/retro/123", {0, image}, db));
}

TEST_F(TempFolder, TestImageStreamer) {
    std::string const record_path{database_path_ + "/record_uuu.db3"};
    cv::Mat const image(480, 720, CV_8UC1);
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    EXPECT_TRUE(database::AddImage("/cam/retro/123", {0, image}, db));
    EXPECT_TRUE(database::AddImage("/cam/retro/123", {2, image}, db));
    EXPECT_TRUE(database::AddImage("/cam/retro/123", {4, image}, db));

    database::ImageStreamer streamer{db, "/cam/retro/123"};
    auto const frame_0{streamer.Next()};
    ASSERT_TRUE(frame_0.has_value());
    EXPECT_EQ(frame_0.value().timestamp_ns, 0);
    EXPECT_EQ(frame_0.value().image.rows, 480);
    EXPECT_EQ(frame_0.value().image.cols, 720);

    EXPECT_TRUE(streamer.Next().has_value());  // Frame 2
    EXPECT_TRUE(streamer.Next().has_value());  // Frame 3, last frame
    EXPECT_FALSE(streamer.Next().has_value());
}

// This simulates the case where we have already processed the images up to a certain time and only want to load the
// images after that point.
TEST_F(TempFolder, TestImageStreamerStartTime) {
    std::string const record_path{database_path_ + "/record_uuu.db3"};
    cv::Mat const image(480, 720, CV_8UC1);
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    EXPECT_TRUE(database::AddImage("/cam/retro/123", {0, image}, db));
    EXPECT_TRUE(database::AddImage("/cam/retro/123", {2, image}, db));
    EXPECT_TRUE(database::AddImage("/cam/retro/123", {4, image}, db));

    database::ImageStreamer streamer{db, "/cam/retro/123", 1};

    auto const frame_0{streamer.Next()};
    ASSERT_TRUE(frame_0.has_value());
    EXPECT_EQ(frame_0.value().timestamp_ns, 2);  // First frame starts at 2ns now

    EXPECT_TRUE(streamer.Next().has_value());
    EXPECT_FALSE(streamer.Next().has_value());
}
