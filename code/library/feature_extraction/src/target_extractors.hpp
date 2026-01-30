#pragma once

#include <optional>

#include <opencv2/opencv.hpp>

#include "feature_extraction/target_extraction.hpp"
#include "types/eigen_types.hpp"

#include "april_tag_cpp_wrapper.hpp"

namespace reprojection::feature_extraction {

class CheckerboardExtractor : public TargetExtractor {
   public:
    explicit CheckerboardExtractor(cv::Size const& pattern_size, double const unit_dimension);

    std::optional<ExtractedTarget> Extract(cv::Mat const& image) const override;
};

class CircleGridExtractor : public TargetExtractor {
   public:
    CircleGridExtractor(cv::Size const& pattern_size, double const unit_dimension, bool const asymmetric);

    std::optional<ExtractedTarget> Extract(cv::Mat const& image) const override;

   private:
    bool asymmetric_;
};

class AprilGrid3Extractor : public TargetExtractor {
   public:
    explicit AprilGrid3Extractor(cv::Size const& pattern_size, double const unit_dimension);

    std::optional<ExtractedTarget> Extract(cv::Mat const& image) const override;

    // WARN(Jack): The corner indices as labeled here to not logically match the order of how they are extracted. You
    // can see this when running the live demo that the indices do not show up in the expected logical row and column
    // order.
    // TODO(Jack): We need a better name that conotates its more complicated function, also calculating the points
    static ArrayXi VisibleGeometry(cv::Size const& pattern_size, std::vector<AprilTagDetection> const& detections);

    static MatrixX3d CornerPositions(ArrayX2i const& indices, double const unit_dimension);

   private:
    // TODO(Jack): Consider making these two extraction functions public and testing them!
    static Matrix42d EstimateExtractionCorners(Matrix3d const& H, int const sqrt_num_bits);

    static Matrix42d RefineCorners(cv::Mat const& image, Matrix42d const& extraction_corners);

    AprilTagFamily tag_family_;
    AprilTagDetector tag_detector_;
};

}  // namespace reprojection::feature_extraction