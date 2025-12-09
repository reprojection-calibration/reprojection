#include "database/database.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "sql.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

CalibrationDatabase::CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only) {
    if (create and read_only) {
        throw std::runtime_error(
            "You requested to open a database object with both options 'create' and 'read_only' true. This is "
            "an invalid combination as creating a database requires writing to it!");
    }

    // TODO(Jack): Consider using sqlite3_errcode for better terminal output https://sqlite.org/c3ref/errcode.html
    int code;
    if (create) {
        // WARN(Jack): Should it be an error if create is true and the database already exists. Is that a problem?
        code = sqlite3_open_v2(db_path.c_str(), &db_,
                               static_cast<int>(SqliteFlag::OpenReadWrite) | static_cast<int>(SqliteFlag::OpenCreate),
                               nullptr);
    } else if (read_only) {
        code = sqlite3_open_v2(db_path.c_str(), &db_, static_cast<int>(SqliteFlag::OpenReadOnly), nullptr);
    } else {
        code = sqlite3_open_v2(db_path.c_str(), &db_, static_cast<int>(SqliteFlag::OpenReadWrite), nullptr);
    }

    if (code != 0) {
        sqlite3_close(db_);
        throw std::runtime_error("Attempted to open database at path - " + db_path + " - but was unsuccessful");
    }

    // WARN(Jack): Is there any circumstance under which the data table creation might fail, and casting to void here
    // instead of explicitly handling the status makes sense?
    static_cast<void>(Sqlite3Tools::Execute(imu_table_sql, db_));
    static_cast<void>(Sqlite3Tools::Execute(images_table_sql, db_));
}

CalibrationDatabase::~CalibrationDatabase() { sqlite3_close(db_); }

[[nodiscard]] bool CalibrationDatabase::AddImuData(std::string const& sensor_name, ImuData const& data) {
    std::string const insert_imu_data_sql{
        InsertImuDataSql(data.timestamp_ns, sensor_name, data.angular_velocity, data.linear_acceleration)};

    return Sqlite3Tools::Execute(insert_imu_data_sql, db_);
}

std::optional<std::set<ImuData>> CalibrationDatabase::GetImuData(std::string const& sensor_name) const {
    std::string const select_imu_sensor_data_sql{SelectImuSensorDataSql(sensor_name)};
    // This callback will be executed on every row returned by the select_imu_sensor_data_sql query.
    auto callback = [](void* data, int, char** argv, char**) -> int {
        auto* const set = static_cast<std::set<ImuData>*>(data);
        set->insert(ImuData{std::stoull(argv[0]),
                            {std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3])},
                            {std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])}});
        return 0;
    };

    std::set<ImuData> data;
    bool const success{Sqlite3Tools::Execute(select_imu_sensor_data_sql, db_, callback, &data)};
    if (not success) {
        // NOTE(Jack): I do not know under what conditions we will ever hit this failure condition, and cannot engineer
        // a test to cover this branch, so we have to supress the code coverage requirement. This is a sign maybe we are
        // missing the point with our design of Execute.
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    return data;
}

// Adopted from https://stackoverflow.com/questions/18092240/sqlite-blob-insertion-c
bool CalibrationDatabase::AddImage(std::string const& sensor_name, uint64_t const timestamp_ns, cv::Mat const& image) {
    std::string const insert_image_sql{InsertImageSql(sensor_name, timestamp_ns)};
    sqlite3_stmt* stmt{nullptr};
    int code{sqlite3_prepare_v2(db_, insert_image_sql.c_str(), -1, &stmt, nullptr)};
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "Add image sqlite3_prepare_v2() failed: " << sqlite3_errmsg(db_) << std::endl;
        return false;
    }

    std::vector<uchar> buffer;
    if (not cv::imencode(".png", image, buffer)) {
        std::cerr << "Failed to encode image as PNG" << std::endl;
        return false;
    }

    // https://stackoverflow.com/questions/1229102/when-to-use-sqlite-transient-vs-sqlite-static
    // Note that the position is not zero indexed here! Therefore 1 corresponds to the first (and only) binding.
    code = sqlite3_bind_blob(stmt, 1, buffer.data(), static_cast<int>(buffer.size()), SQLITE_STATIC);
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "Add image sqlite3_bind_blob() failed: " << sqlite3_errmsg(db_) << std::endl;
        return false;
    }

    code = sqlite3_step(stmt);
    if (code != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "Add image sqlite3_step() failed: " << sqlite3_errmsg(db_) << std::endl;
        return false;
    }

    sqlite3_finalize(stmt);

    return true;
}

};  // namespace reprojection::database