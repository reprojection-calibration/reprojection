#include "database/image_interface.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(DatabaseImageInterface, TestAddImage) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    cv::Mat const image(10, 20, CV_8UC1);
    EXPECT_NO_THROW(database::AddImage({{0, "/cam/retro/123"}, image}, db));
}

TEST(DatabaseImageInterface, TestAddImageHeaderOnly) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    EXPECT_NO_THROW(database::AddImage(FrameHeader{0, "/cam/retro/123"}, db));
}

TEST(DatabaseImageInterface, TestAddImageError) {
    // Here we make a database and then close it, and then open another database that is read only and cannot have an
    // image written to it.
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};
    db = std::make_shared<database::CalibrationDatabase>(temp_file.Path(), false, true);

    cv::Mat const image(10, 20, CV_8UC1);
    EXPECT_THROW(database::AddImage({{0, "/cam/retro/123"}, image}, db), std::runtime_error);
}

TEST(DatabaseImageInterface, TestAddImageHeaderOnlyError) {
    TemporaryFile const temp_file{".db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};
    db = std::make_shared<database::CalibrationDatabase>(temp_file.Path(), false, true);

    EXPECT_THROW(database::AddImage(FrameHeader{0, "/cam/retro/123"}, db), std::runtime_error);
}

TEST(DatabaseImageInterface, TestImageStreamer) {
    TemporaryFile const temp_file{".db3"};
    cv::Mat const image(10, 20, CV_8UC1);
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    EXPECT_NO_THROW(database::AddImage({{0, "/cam/retro/123"}, image}, db));
    EXPECT_NO_THROW(database::AddImage({{2, "/cam/retro/123"}, image}, db));
    EXPECT_NO_THROW(database::AddImage({{4, "/cam/retro/123"}, image}, db));

    database::ImageStreamer streamer{db, "/cam/retro/123"};
    auto const frame_0{streamer.Next()};
    ASSERT_TRUE(frame_0.has_value());
    EXPECT_EQ(frame_0.value().header.timestamp_ns, 0);
    EXPECT_EQ(frame_0.value().header.sensor_name, "/cam/retro/123");
    EXPECT_EQ(frame_0.value().image.rows, 10);
    EXPECT_EQ(frame_0.value().image.cols, 20);

    EXPECT_TRUE(streamer.Next().has_value());  // Frame 2
    EXPECT_TRUE(streamer.Next().has_value());  // Frame 3, last frame
    EXPECT_FALSE(streamer.Next().has_value());
}

// This simulates the case where we have already processed the images up to a certain time and only want to load the
// images after that point.
TEST(DatabaseImageInterface, TestImageStreamerStartTime) {
    TemporaryFile const temp_file{".db3"};
    cv::Mat const image(10, 20, CV_8UC1);
    auto db{std::make_shared<database::CalibrationDatabase>(temp_file.Path(), true, false)};

    EXPECT_NO_THROW(database::AddImage({{0, "/cam/retro/123"}, image}, db));
    EXPECT_NO_THROW(database::AddImage({{2, "/cam/retro/123"}, image}, db));
    EXPECT_NO_THROW(database::AddImage({{4, "/cam/retro/123"}, image}, db));

    database::ImageStreamer streamer{db, "/cam/retro/123", 1};

    auto const frame_0{streamer.Next()};
    ASSERT_TRUE(frame_0.has_value());
    EXPECT_EQ(frame_0.value().header.timestamp_ns, 2);  // First frame starts at 2ns now

    EXPECT_TRUE(streamer.Next().has_value());
    EXPECT_FALSE(streamer.Next().has_value());
}
