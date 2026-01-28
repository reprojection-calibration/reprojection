#include "target_options.hpp"

#include <toml++/toml.hpp>

#include "target_enum_parsing.hpp"
#include "toml_helpers.hpp"

namespace reprojection::config {

bool ValidateTargetConfig(toml::table const& target_cfg) {
    // TODO(Jack): Why does my use of .merge() below not allow this to be const? Makes no sense.
    std::map<std::string, DataType> required_keys{
        {"pattern_size", DataType::Array},
        {"type", DataType::String},

    };

    auto validation_error{ValidateRequiredKeys(target_cfg, required_keys)};
    if (validation_error) {
        // TODO REFACTOR THE REAL ERROR
        return false;
    }

    std::map<std::string, DataType> optional_keys{
        {"circle_grid", DataType::Table},
        {"circle_grid.asymmetric", DataType::Boolean},
        {"unit_dimension", DataType::FloatingPoint},
    };

    // TODO(Jack): I wish there was a better way to merge the maps here and express the semantics of what is required,
    //  what is optional, and how the combination of the two is what is possible.
    optional_keys.merge(required_keys);
    std::map<std::string, DataType> const possible_keys{optional_keys};

    validation_error = ValidatePossibleKeys(target_cfg, possible_keys);
    if (validation_error) {
        // TODO REFACTOR THE REAL ERROR
        return false;
    }

    return true;
}

}  // namespace reprojection::config
