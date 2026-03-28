#include "formatting.hpp"

namespace reprojection::logging {

std::string CompactPrettyJson(std::string const& input) {
    std::string out;
    out.reserve(input.size());

    bool in_string = false;
    bool escape = false;

    for (size_t i = 0; i < input.size(); ++i) {
        char c = input[i];

        if (escape) {
            out += c;
            escape = false;
            continue;
        }

        if (c == '\\') {
            out += c;
            escape = true;
            continue;
        }

        if (c == '"') {
            in_string = !in_string;
            out += "'";
            continue;
        }

        if (!in_string) {
            if (std::isspace(static_cast<unsigned char>(c))) continue;

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
}

}  // namespace reprojection::logging
