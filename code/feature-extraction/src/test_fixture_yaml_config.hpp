#pragma once

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace reprojection_calibration::feature_extraction {

class YamlConfigTestFixture : public ::testing::Test {
   public:
    YamlConfigTestFixture()
        : checkerboard_{BuildBaseNode("checkerboard", 3, 4, 0.05)},
          april_grid3_{BuildBaseNode("april_grid3", 3, 4, 0.05)} {
        // NOTE(Jack): Because circle grid is the only one with custom options we just build it here manually in the
        // constructor. If the target specifications ever get more complex we can build methods to do this more
        // discretely.
        circle_grid_ = BuildBaseNode("circle_grid", 3, 4, 0.05);
        circle_grid_["circle_grid_options"]["asymmetric"] = false;
    }

    YAML::Node checkerboard_;
    YAML::Node circle_grid_;
    YAML::Node april_grid3_;

   private:
    YAML::Node BuildBaseNode(std::string const& target_type, int const rows, int const cols,
                             double const unit_dimension) {
        YAML::Node base_node;
        base_node["type"] = target_type;
        base_node["pattern_size"].push_back(rows);
        base_node["pattern_size"].push_back(cols);
        base_node["unit_dimension"] = unit_dimension;

        return base_node;
    }
};

}  // namespace reprojection_calibration::feature_extraction