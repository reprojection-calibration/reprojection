
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
}

CalibrationDatabase::~CalibrationDatabase() { sqlite3_close(db_); }

[[nodiscard]] bool CalibrationDatabase::AddImuData(std::string const& sensor_name, ImuData const& data) {
    // TODO(Jack): This is hacky! We need to find a principle way to create all data tables, not attempt to recreate it
    // everytime we want to add data!
    bool success{Sqlite3Tools::Execute(imu_table_sql, db_)};
    if (not success) {
        return false;
    }

    std::string const insert_imu_data_sql{
        InsertImuDataSql(data.timestamp_ns, sensor_name, data.angular_velocity, data.linear_acceleration)};

    return Sqlite3Tools::Execute(insert_imu_data_sql, db_);
}

// TODO(Jack): Return a variant here to indicate success or failure
std::set<ImuData> CalibrationDatabase::GetImuData(std::string const& sensor_name) {
    std::string const select_imu_sensor_data_sql{SelectImuSensorDataSql(sensor_name)};

    auto callback = [](void* data, int, char** argv, char**) -> int {
        auto* set = reinterpret_cast<std::set<ImuData>*>(data);

        set->insert(ImuData{std::stoull(argv[0]),
                            {std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3])},
                            {std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])}});

        return 0;
    };

    std::set<ImuData> data;
    (void)Sqlite3Tools::Execute(select_imu_sensor_data_sql, db_, callback, &data);  // REMOVE VOID CAST, RETURN VARIANT

    return data;
}

};  // namespace reprojection::database