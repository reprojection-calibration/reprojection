set(LIBRARY_NAME "database")

# NOTE(Jack): Because we want one single source of truth for both the library and python tooling we store the sql files
# under `code/sql`, outside of the cmake package. Therefore just like for the proto files below we need to backspace two
# directories "../../${sql}" to get to the actual file. If this is a bad design to hardcode reference paths outisde of
# the package we will find out one day in the future!
# WARN(Jack): If you change something in an .sql file you NEED to reload the cmake project! Otherwise the change will
# not be reflected in the generated header file.
set(SQL_FILES
        sql/calibration_steps_delete.sql
        sql/calibration_steps_select.sql
        sql/calibration_steps_table.sql
        sql/calibration_steps_upsert.sql
        sql/camera_info_insert.sql
        sql/camera_info_select.sql
        sql/camera_info_table.sql
        sql/camera_intrinsics_insert.sql
        sql/camera_intrinsics_select.sql
        sql/camera_intrinsics_table.sql
        sql/extracted_target_insert.sql
        sql/extracted_targets_select.sql
        sql/extracted_targets_table.sql
        sql/image_insert.sql
        sql/images_select.sql
        sql/images_table.sql
        sql/imu_data_insert.sql
        sql/imu_data_select.sql
        sql/imu_data_table.sql
        sql/poses_insert.sql
        sql/poses_select.sql
        sql/poses_table.sql
        sql/reprojection_error_insert.sql
        sql/reprojection_error_table.sql
)

foreach (sql ${SQL_FILES})
    get_filename_component(name ${sql} NAME_WE)
    file(READ "../../${sql}" contents)

    set(SQL_STRING_ENTRIES
            "${SQL_STRING_ENTRIES}\nconstexpr char const * const ${name} = R\"sql(\n${contents}\n)sql\";\n")
endforeach ()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sql.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/sql.hpp
        @ONLY
)

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
        src/image_interface.cpp
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
        protobuf::libprotobuf
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
        test/database_read.test.cpp
        test/database_remove.test.cpp
        test/database_write.test.cpp
        test/image_interface.test.cpp
)
AddTests()