#pragma once

#include <map>
#include <stdexcept>
#include <string>

namespace reprojection::config {

template <typename T>
concept IsEnum = std::is_enum_v<T>;

template <typename T_Enum, auto T_Parser>
concept IsCeresConverter = requires(std::string value, T_Enum* type) {
    { T_Parser(value, type) } -> std::same_as<bool>;
};

template <typename T_Enum, auto T_Parser>
    requires IsEnum<T_Enum> and IsCeresConverter<T_Enum, T_Parser>
T_Enum CeresEnumToString(std::string const& enum_string) {
    T_Enum output;
    bool const parsed{T_Parser(enum_string, &output)};

    if (not parsed) {
        throw std::runtime_error("String to enum conversion failed for: " + enum_string);
    }

    return output;
}

}  // namespace reprojection::config