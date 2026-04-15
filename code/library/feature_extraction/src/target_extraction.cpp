#include "feature_extraction/target_extraction.hpp"

#include "config/config_validation.hpp"
#include "logging/logging.hpp"
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
std::unique_ptr<TargetExtractor> CreateTargetExtractor(toml::table const& target_config) {
    if (auto const error_msg{config::ValidateTargetConfig(target_config)}) {
        throw std::runtime_error(error_msg->msg);
    }

    std::string type_str{target_config["type"].as_string()->get()};
    TargetType const type{ToTargetType(type_str)};
    cv::Size const pattern_size{static_cast<int>(target_config["pattern_size"].as_array()->at(1).as_integer()->get()),
                                static_cast<int>(target_config["pattern_size"].as_array()->at(0).as_integer()->get())};

    // NOTE(Jack) For optional parameters like unit_dimension we need to check if they are in the table first.
    double unit_dimension{1.0};  // Sensible default
    if (auto const node{target_config["unit_dimension"]}) {
        unit_dimension = node.as_floating_point()->get();
    }

    if (type == TargetType::Checkerboard) {
        return std::make_unique<CheckerboardExtractor>(pattern_size, unit_dimension);
    } else if (type == TargetType::CircleGrid) {
        bool asymmetric{false};
        // TODO(Jack): Something I do not like here is now that we have this key sequence hardcoded in two places.
        // Once
        //  in the config loading checking logic and once here. We should figure out how to have one central source
        //  of truth that holds all possible user interactive config keys.
        if (auto const node{target_config.at_path("circle_grid.asymmetric")}) {
            asymmetric = node.as_boolean()->get();
        }
        return std::make_unique<CircleGridExtractor>(pattern_size, unit_dimension, asymmetric);
    } else if (type == TargetType::AprilGrid3) {
        return std::make_unique<AprilGrid3Extractor>(pattern_size, unit_dimension);
    } else {
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - CreateTargetExtractor() invalid feature extractor type: " +  // LCOV_EXCL_LINE
            type_str);  // LCOV_EXCL_LINE
    }
}

// TODO/WARN(Jack): This functions expects that the image has three channels but that is not enforced anywhere here.
// What would happen if this was fed a grayscale image or a float image?
void DrawTarget(ExtractedTarget const& target, cv::Mat const& img) {
    MatrixX2d const& pixels{target.bundle.pixels};
    ArrayX2i const& indices{target.indices};

    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        cv::circle(img, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), 1, cv::Scalar(0, 255, 0), 5, cv::LINE_8);

        std::string const text{"(" + std::to_string(indices.row(i)[0]) + ", " + std::to_string(indices.row(i)[1]) +
                               ")"};
        cv::putText(img, text, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), cv::FONT_HERSHEY_COMPLEX, 0.4,
                    cv::Scalar(255, 255, 255), 1);
    }
}

}  // namespace reprojection::feature_extraction