#include "feature_extraction/target_extraction.hpp"

#include "config/config_validation.hpp"
#include "logging/logging.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

#include "target_extractors.hpp"
#include "utilities.hpp"

namespace reprojection::feature_extraction {

namespace {

auto const log{logging::Get("feature_extraction")};

}

// TODO(Jack): If we run into trouble one day with different opencv image formats and depths we should extract the
// image
//  type processing code here and unit test it.
std::optional<ExtractedTarget> TargetExtractor::Extract(cv::Mat const& img) {
    log->debug("{{'input_image': {{'height': {}, 'width': {}, 'depth': '{}', 'channels': {}}}}}", img.rows, img.cols,
               cv::depthToString(img.depth()), img.channels());

    cv::Mat const img_gray{ToGray(img)};
    auto const extracted_target{ExtractImplementation(img_gray)};

    log->debug("{{'successful_extraction': {}, 'num_features': {}}}", extracted_target.has_value(),
               extracted_target ? extracted_target.value().indices.rows() : 0);

    return extracted_target;
}

// TODO(Jack): This is a slightly controversial decision to let the toml::table type escape outside of the config
//  package. However we need to allow for some dynamic behaviour (ex. circle grid asymmetric parameter), and the
//  table is a type which lets us easily do this, when compared to making structs with base classes etc.
// TODO(Jack): Somewhere in the process we should check that pattern size is a length two array of ints. We access
//  it here without checking and therefore risk scary failures.
std::unique_ptr<TargetExtractor> CreateTargetExtractor(TargetInfo const& target_info) {
    cv::Size const pattern_size{target_info.width, target_info.height};

    // ADD UNIT DIMENSION TO TARGET_INFO!!!
    // ADD UNIT DIMENSION TO TARGET_INFO!!!
    // ADD UNIT DIMENSION TO TARGET_INFO!!!
    // ADD UNIT DIMENSION TO TARGET_INFO!!!
    // NOTE(Jack) For optional parameters like unit_dimension we need to check if they are in the table first.
    double unit_dimension{1.0};

    if (target_info.target_type == TargetType::Checkerboard) {
        return std::make_unique<CheckerboardExtractor>(pattern_size, unit_dimension);
    } else if (target_info.target_type == TargetType::CircleGrid) {
        return std::make_unique<CircleGridExtractor>(pattern_size, unit_dimension, target_info.asymmetric);
    } else if (target_info.target_type == TargetType::Aprilgrid3) {
        return std::make_unique<Aprilgrid3Extractor>(pattern_size, unit_dimension);
    } else {
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - CreateTargetExtractor() invalid feature extractor type: " +  // LCOV_EXCL_LINE
            ToString(target_info.target_type));  // LCOV_EXCL_LINE
    }
}

// TODO/WARN(Jack): This functions expects that the image has three channels but that is not enforced anywhere here.
// What would happen if this was fed a grayscale image or a float image?
void DrawTarget(ExtractedTarget const& target, cv::Mat const& img) {
    MatrixX2d const& pixels{target.bundle.pixels};
    ArrayX2i const& indices{target.indices};

    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        cv::circle(img, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), 5, cv::Scalar(0, 255, 0), 1, cv::LINE_8);

        std::string const text{"(" + std::to_string(indices.row(i)[0]) + ", " + std::to_string(indices.row(i)[1]) +
                               ")"};
        cv::putText(img, text, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), cv::FONT_HERSHEY_COMPLEX, 0.4,
                    cv::Scalar(255, 255, 255), 1);
    }
}

}  // namespace reprojection::feature_extraction