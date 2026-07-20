set(LIBRARY_NAME "database")

# NOTE(Jack): Because we want one single source of truth for both the library and python tooling we store the sql files
# under `code/sql`, outside of the cmake package. Therefore just like for the proto files below we need to backspace two
# directories "../../${sql}" to get to the actual file. If this is a bad design to hardcode reference paths outisde of
# the package we will find out one day in the future!
# WARN(Jack): If you change something in an .sql file you NEED to reload the cmake project! Otherwise the change will
# not be reflected in the generated header file.
set(SQL_FILES
        calibration_steps_delete.sql
        calibration_steps_select.sql
        calibration_steps_table.sql
        calibration_steps_upsert.sql
        camera_info_insert.sql
        camera_info_select.sql
        camera_info_table.sql
        camera_intrinsics_insert.sql
        camera_intrinsics_select.sql
        camera_intrinsics_table.sql
        entity_insert_or_ignore.sql
        entity_table.sql
        extracted_target_insert.sql
        extracted_targets_select.sql
        extracted_targets_table.sql
        extrinsics_insert.sql
        extrinsics_select.sql
        extrinsics_table.sql
        gravity_insert.sql
        gravity_select.sql
        gravity_table.sql
        image_insert.sql
        images_select.sql
        images_table.sql
        imu_data_insert.sql
        imu_data_select.sql
        imu_data_table.sql
        imu_errors_insert.sql
        imu_errors_select.sql
        imu_errors_table.sql
        poses_insert.sql
        poses_select.sql
        poses_table.sql
        reprojection_error_insert.sql
        reprojection_error_table.sql
        spline_control_points_count.sql
        spline_control_points_insert.sql
        spline_control_points_select.sql
        spline_control_points_table.sql
        spline_time_handler_insert.sql
        spline_time_handler_select.sql
        spline_time_handler_table.sql
        target_info_insert.sql
        target_info_select.sql
        target_info_table.sql
)

set(SQL_DIR "$ENV{REPROJECTION_SQL_DIR}")

foreach (sql ${SQL_FILES})
    get_filename_component(name ${sql} NAME_WE)
    # TODO(Jack): Having to escape back three directories definitely looks a little hacky... but it gets the job done,
    # at least in the clion dev toolchain process.
    file(READ "../../../${SQL_DIR}/${sql}" contents)

    set(SQL_STRING_ENTRIES
            "${SQL_STRING_ENTRIES}constexpr char const* const ${name} = R\"sql(\n${contents}\n)sql\";\n")
endforeach ()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sql.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/sql.hpp
        @ONLY
)

#####

set(SQL2_FILES
        assets_select.sql
        assets_insert.sql
        assets_table.sql
        images_insert.sql
        images_table.sql
        recordings_select.sql
        recordings_insert.sql
        recordings_table.sql
        runs_select.sql
        runs_insert.sql
        runs_table.sql
        steps_delete.sql
        steps_insert.sql
        steps_insert_id.sql
        steps_select.sql
        steps_table.sql
)

set(SQL2_DIR "code/sql2")

foreach (sql ${SQL2_FILES})
    get_filename_component(name ${sql} NAME_WE)
    file(READ "../../../${SQL2_DIR}/${sql}" contents)

    set(SQL_STRING_ENTRIES
            "${SQL_STRING_ENTRIES}constexpr char const* const ${name} = R\"sql(\n${contents}\n)sql\";\n")
endforeach ()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sql.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/sql2.hpp
        @ONLY
)

#####

# Protobuf logic adopted from https://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers
# TODO(Jack): Should we compile database_proto as static like in the example linked above?
# TODO(Jack): Is there anyway to have the generated code go to the "generated/" folder? It would add some consistency
# with the generated sql code which we are otherwise lacking.
# TODO(Jack): This path here scares me because now we are hardcoding a relative path to an asset that exists outside
# of the cmake package. This smells! See path above about sql files too.
protobuf_generate_cpp(proto_sources proto_headers
        "../../proto/extracted_target.proto"
)

set(SRC_FILES
        src/calibration_database.cpp
        src/database_read.cpp
        src/database_remove.cpp
        src/database_write.cpp
        src/serialization.cpp
        src/sqlite_wrappers.cpp
        src/toml_converters.cpp
)
set(SRC_FILES
        ${SRC_FILES}
        ${proto_sources}
        ${proto_headers}
)
set(INCLUDE_DIRECTORIES
        ${CMAKE_BINARY_DIR}/database # Generated sql header files are found here
        ${OpenCV_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${OpenCV_LIBS}
        Eigen3::Eigen
        SQLite::SQLite3
        geometry # Required because of spline interface
        protobuf::libprotobuf
        spline
        tomlplusplus::tomlplusplus
        types
        types_internal
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        src/query_runner.test.cpp
        src/serialization.test.cpp
        src/statement_executor.test.cpp
        src/sqlite3_helpers.test.cpp
        src/sqlite_wrappers.test.cpp
        src/toml_converters.test.cpp
        test/calibration_database.test.cpp
        test/calibration_database2.test.cpp
        test/database_read_write.test.cpp
        test/database_remove.test.cpp
)
AddTests()