#include "feature_extraction/target_extraction.hpp"

#include <stdexcept>

#include "target_extractors.hpp"

namespace reprojection_calibration::feature_extraction {

TargetType ToEnum(std::string const& target_type) {
    if (target_type == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (target_type == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (target_type == "april_grid3") {
        return TargetType::AprilGrid3;
    } else {
        throw std::runtime_error("The requested target type is not part of the TargetType enum parsing.");
    }
}

std::unique_ptr<TargetExtractor> CreateTargetExtractor(YAML::Node const& target_config) {
    if (not target_config["type"]) {
        // TODO(Jack): Does the yaml-cpp library provide us a better error handling mechanism?
        throw std::runtime_error("The target type was not specified.");
    }
    TargetType const type{ToEnum(target_config["type"].as<std::string>())};

    if (not(target_config["pattern_size"] and target_config["pattern_size"].IsSequence())) {
        throw std::runtime_error("The target pattern_size was not specified.");
    }
    cv::Size const pattern_size{target_config["pattern_size"][1].as<int>(), target_config["pattern_size"][0].as<int>()};

    if (not target_config["unit_dimension"]) {
        throw std::runtime_error("The target unit_dimension was not specified.");
    }
    double const unit_dimension{target_config["unit_dimension"].as<double>()};  // comes from config file in the future

    if (type == TargetType::Checkerboard) {
        return std::make_unique<CheckerboardExtractor>(pattern_size, unit_dimension);
    } else if (type == TargetType::CircleGrid) {
        if (not target_config["circle_grid_options"]["asymmetric"]) {
            throw std::runtime_error("The target cirle_grid_options.asymmetric was not specified.");
        }
        bool const asymmetric{target_config["circle_grid_options"]["asymmetric"].as<bool>()};

        return std::make_unique<CircleGridExtractor>(pattern_size, unit_dimension, asymmetric);
    } else {
        return std::make_unique<AprilGrid3Extractor>(pattern_size, unit_dimension);
    }
}

}  // namespace reprojection_calibration::feature_extraction