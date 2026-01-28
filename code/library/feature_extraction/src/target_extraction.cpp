#include "feature_extraction/target_extraction.hpp"

#include <stdexcept>

#include "target_extractors.hpp"
#include "types/enums.hpp"

namespace reprojection::feature_extraction {

std::unique_ptr<TargetExtractor> CreateTargetExtractor(toml::table const& target_config) {
    (void)target_config;
    return nullptr;
}

}  // namespace reprojection::feature_extraction