#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <optional>

#include "april_tag_cpp_wrapper.hpp"
#include "feature_extraction/target_extraction.hpp"

namespace reprojection_calibration::feature_extraction {

class CheckerboardExtractor : public TargetExtractor {
   public:
    explicit CheckerboardExtractor(cv::Size const& pattern_size, double const unit_dimension);

    std::optional<FeatureFrame> Extract(cv::Mat const& image) const override;
};

class CircleGridExtractor : public TargetExtractor {
   public:
    CircleGridExtractor(cv::Size const& pattern_size, double const unit_dimension, bool const asymmetric);

    std::optional<FeatureFrame> Extract(cv::Mat const& image) const override;

   private:
    bool asymmetric_;
};

class AprilGrid3Extractor : public TargetExtractor {
   public:
    explicit AprilGrid3Extractor(cv::Size const& pattern_size, double const unit_dimension);

    std::optional<FeatureFrame> Extract(cv::Mat const& image) const override;

    // WARN(Jack): The corner indices as labeled here to not logically match the order of how they are extracted. You
    // can see this when running the live demo that the indices do not show up in the expected logical row and column
    // order.
    // TODO(Jack): We need a better name that conotates its more complicated function, also calculating the points
    static Eigen::ArrayXi VisibleGeometry(cv::Size const& pattern_size,
                                          std::vector<AprilTagDetection> const& detections);

    static Eigen::MatrixX3d CornerPositions(Eigen::ArrayX2i const& indices, double const unit_dimension);

   private:
    // TODO(Jack): Consider making these two extraction functions public and testing them!
    static Eigen::Matrix<double, 4, 2> EstimateExtractionCorners(Eigen::Matrix3d const& H, int const sqrt_num_bits);

    static Eigen::Matrix<double, 4, 2> RefineCorners(cv::Mat const& image,
                                                     Eigen::Matrix<double, 4, 2> const& extraction_corners);

    AprilTagFamily tag_family_;
    AprilTagDetector tag_detector_;
};

}  // namespace reprojection_calibration::feature_extraction