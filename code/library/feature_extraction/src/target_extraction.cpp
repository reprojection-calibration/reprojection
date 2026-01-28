#include "feature_extraction/target_extraction.hpp"

#include "../../config/src/enum_string_converters.hpp"
#include "config/target_options.hpp"
#include "target_extractors.hpp"
#include "types/enums.hpp"

namespace reprojection::feature_extraction {

// TODO(Jack): This is a slightly controversial decision to let the toml::table type escape outside of the config
//  package. However we need to allow for some dynamic behaviour (ex. circle grid asymmetric parameter), and the table
//  is a type which lets us easily do this, when compared to making structs with base classes etc.
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
        if (auto const node{target_config.at_path("circle_grid.asymmetric")}) {
            asymmetric = node.as_boolean()->get();
        }
        return std::make_unique<CircleGridExtractor>(pattern_size, unit_dimension, asymmetric);
    } else if (type == TargetType::AprilGrid3) {
        return std::make_unique<AprilGrid3Extractor>(pattern_size, unit_dimension);
    } else {
        throw std::runtime_error("CreateTargetExtractor() invalid feature extractor type: " +
                                 type_str);  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::feature_extraction