#include "application/database.hpp"

#include "database/calibration_database.hpp"

namespace reprojection::application {

std::variant<DbPtr, DbErrorMsg> Open(fs::path const& workspace, fs::path const& data_source) {
    if (std::error_code code; not fs::is_directory(workspace, code)) {
        // TODO ERROR CODE PROVIDES AT LEAST STRING STREAM OPERATOR THAT DOES EXACTLY WHAT WE DO HERE AND BELOW
        //  MANUALLY!!!
        return DbErrorMsg{"Provided workspace path: '" + workspace.string() +
                          "' is not a valid directory - error code (" + std::to_string(code.value()) +
                          ") with description '" + code.message() + "'"};
    }
    if (std::error_code code; not fs::is_regular_file(data_source, code)) {
        return DbErrorMsg{"Provided data source path: '" + data_source.string() +
                          "' is not a valid file - error code (" + std::to_string(code.value()) +
                          ") with description '" + code.message() + "'"};
    }

    fs::path const db_path{workspace / (data_source.stem().string() + ".db3")};
    if (fs::exists(db_path)) {
        return std::make_shared<database::CalibrationDatabase>(db_path, false, false);
    }

    return std::make_shared<database::CalibrationDatabase>(db_path, true, false);
}

}  // namespace reprojection::application