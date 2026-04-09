#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(FeatureExtractionUtilities, TestToGray) {
    cv::Mat img_float;
    cv::Mat(100, 100, CV_32F).convertTo(img_float, -1);

    cv::Mat img_gray{feature_extraction::ToGray(img_float)};
    EXPECT_EQ(img_gray.depth(), CV_8U);
    EXPECT_EQ(img_gray.channels(), 1);

    cv::Mat const img_bgr{cv::Mat::ones(100, 100, CV_8UC3)};
    img_gray = feature_extraction::ToGray(img_bgr);
    EXPECT_EQ(img_gray.depth(), CV_8U);
    EXPECT_EQ(img_gray.channels(), 1);

    cv::Mat const img_bgra{cv::Mat::ones(100, 100, CV_8UC4)};
    img_gray = feature_extraction::ToGray(img_bgra);
    EXPECT_EQ(img_gray.depth(), CV_8U);
    EXPECT_EQ(img_gray.channels(), 1);
}

TEST(FeatureExtractionUtilities, TestAlternatingSum) {
    double const sum_1{feature_extraction::AlternatingSum(2, 0.5, 0.2)};
    EXPECT_EQ(sum_1, 0.5 + 0.2);

    double const sum_2{feature_extraction::AlternatingSum(3, 0.5, 0.2)};
    EXPECT_EQ(sum_2, 0.5 + 0.2 + 0.5);

    double const sum_null{feature_extraction::AlternatingSum(0, 0.5, 0.2)};
    EXPECT_EQ(sum_null, 0);
}