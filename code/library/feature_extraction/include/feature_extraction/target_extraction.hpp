#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>

#include "types/algorithm_types.hpp"
#include "types/eigen_types.hpp"

#include <opencv2/opencv.hpp>
#include <toml++/toml.hpp>

namespace reprojection::feature_extraction {

// NOTE(Jack): For detectors which can only detect "whole" boards the Extract() method will simply return the points and
// indices in their entirety. For targets which can have partial detections (ex. AprilGrid3) their Extract() method will
// mask out the indices and points which were visible and only return those. Never assume a target is whole! Always use
// the indices.
class TargetExtractor {
   public:
    TargetExtractor(cv::Size const& pattern_size, const double unit_dimension)
        : pattern_size_{pattern_size}, unit_dimension_{unit_dimension} {}

    virtual ~TargetExtractor() = default;

    virtual std::optional<ExtractedTarget> Extract(cv::Mat const& image) const = 0;

   protected:
    cv::Size pattern_size_;
    double unit_dimension_;
    ArrayX2i point_indices_;
    MatrixX3d points_;
};

std::unique_ptr<TargetExtractor> CreateTargetExtractor(toml::table const& target_config);

}  // namespace reprojection::feature_extraction