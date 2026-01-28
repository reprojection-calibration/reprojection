#include <gtest/gtest.h>

#include <optional>
#include <string>

namespace reprojection {

enum class DataType { Array, Double, Integer, String };

enum class ParseErrorType { UnknownKey, IncorrectType };

struct ParseError {
    ParseErrorType error;
    std::string_view msg;
};

}  // namespace reprojection

namespace reprojection::config {

std::optional<ParseError> ValidateToml(toml::table const& table) {}

}  // namespace reprojection::config

using namespace reprojection;

TEST(ConfigTomlHelpers, Test) {
    std::map<std::string, DataType> const required_keys{{"pattern_size", DataType::Array},
                                                        {"unit_dimension", DataType::Double},
                                                        {"num_threads", DataType::Integer},
                                                        {"minimizer_type", DataType::String}};

    EXPECT_EQ(1, 2);
}