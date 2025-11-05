#include "demos/image_source.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

using namespace reprojection;

// NOTE(Jack): We cannot simulate a USB camera or real device, therefore we test it using a video file. The opencv video
// capture supports both input modalities with the same interface.
TEST(DemosImageSource, VideoCaptureMp4) {
    std::string const folder{"test/video_capture/feed/"};
    std::filesystem::create_directories(folder);
    cv::Mat const blank_image{cv::Mat::zeros(10, 10, CV_8UC1)};

    // Write two video frames into a mp4 video
    cv::VideoWriter writer;
    if (not writer.open(folder + "video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0,
                        cv::Size{blank_image.cols, blank_image.rows}, false)) {
        std::cerr << "Error: Could not open the video writer!" << std::endl;
    }
    writer.write(blank_image);
    writer.write(blank_image);
    writer.release();

    // Load the video and test that we get two frames
    demos::VideoCapture image_feed{folder + "video.mp4"};

    cv::Mat loaded_image{image_feed.GetImage()};
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 0);  // Third attempt at loading returns empty image.

    std::filesystem::remove(folder + "video.mp4");
}

TEST(DemosImageSource, VideoCaptureError) {
    EXPECT_THROW(demos::VideoCapture image_feed{"non_existent_video.mp4"}, std::runtime_error);
}

TEST(DemosImageSource, TestImageFolder) {
    std::string const folder{"test/folder/feed/"};
    std::filesystem::create_directories(folder);

    cv::Mat const blank_image{cv::Mat::zeros(10, 10, CV_8UC1)};
    cv::imwrite(folder + "01.png", blank_image);
    cv::imwrite(folder + "02.png", blank_image);

    // Load the folder and check that we get two frames
    demos::ImageFolder image_feed{folder};

    cv::Mat loaded_image{image_feed.GetImage()};
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 0);  // Third attempt at loading returns empty image.

    std::filesystem::remove_all(folder);
}
