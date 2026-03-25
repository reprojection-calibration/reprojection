#include "application/database.hpp"

namespace reprojection::application {

std::variant<DbPtr, DbErrorMsg> Open(fs::path const& workspace, fs::path const& data_source) {
    if (std::error_code code; not fs::is_directory(workspace, code)) {
        return DbErrorMsg{"Provided workspace path: '" + workspace.string() +
                          "' is not a valid directory - error code (" + std::to_string(code.value()) +
                          ") with description '" + code.message() + "'"};
    }
    if (std::error_code code; not fs::is_regular_file(data_source, code)) {
        return DbErrorMsg{"Provided data source path: '" + data_source.string() +
                          "' is not a valid file - error code (" + std::to_string(code.value()) +
                          ") with description '" + code.message() + "'"};
    }

    return nullptr;
}

}  // namespace reprojection::application