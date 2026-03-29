#include "formatting.hpp"

namespace reprojection::logging {

std::string CompactPrettyJson(std::string const& input) {
    std::string out;
    out.reserve(std::size(input));

    bool in_string{false};

    for (size_t i = 0; i < input.size(); ++i) {
        char const c{input[i]};

        if (c == '"') {
            in_string = not in_string;
            out += "'";
            continue;
        }

        if (not in_string) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                continue;
            }

            // Add normalized spacing so it is more readable
            if (c == ':') {
                out += ": ";
                continue;
            }
            if (c == ',') {
                out += ", ";
                continue;
            }
        }

        out += c;
    }

    return out;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::logging
