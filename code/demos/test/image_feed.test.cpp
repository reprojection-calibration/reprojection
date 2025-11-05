#include "demos/image_feed.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

using namespace reprojection;

TEST(DemosImageFeed, VideoCaptureFeedMp4) {
    std::string const folder{"test/video_capture/feed/"};
    std::filesystem::create_directories(folder);
    cv::Mat const blank_image{cv::Mat::zeros(10, 10, CV_8UC1)};

    cv::VideoWriter writer;
    if (not writer.open(folder + "video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0,
                        cv::Size{blank_image.cols, blank_image.rows}, false)) {
        std::cerr << "Error: Could not open the video writer!" << std::endl;
    }
    // Write two video frames
    writer.write(blank_image);
    writer.write(blank_image);
    writer.release();

    demos::VideoCaptureFeed image_feed{folder + "video.mp4"};

    cv::Mat loaded_image{image_feed.GetImage()};
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 0);  // Third attempt at loading returns empty image.

}

TEST(DemosImageFeed, TestFolderFeed) {
    std::string const folder{"test/folder/feed/"};
    std::filesystem::create_directories(folder);

    cv::Mat const blank_image{cv::Mat::zeros(10, 10, CV_8UC1)};
    cv::imwrite(folder + "01.png", blank_image);
    cv::imwrite(folder + "02.png", blank_image);

    demos::FolderFeed image_feed{folder};

    cv::Mat loaded_image{image_feed.GetImage()};
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 100);
    loaded_image = image_feed.GetImage();
    EXPECT_EQ(loaded_image.rows * loaded_image.cols, 0);  // Third attempt at loading returns empty image.

    std::filesystem::remove_all(folder);
}
