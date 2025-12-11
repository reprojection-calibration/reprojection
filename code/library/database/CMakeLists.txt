set(LIBRARY_NAME "database")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(SQLite3 REQUIRED)

# WARN(Jack): If you change something in an .sql file you NEED to reload the cmake project! Otherwise the change will
# not be reflected in the generated header file.
set(SQL_FILES
        sql/extracted_target_insert.sql
        sql/extracted_targets_select.sql
        sql/extracted_targets_table.sql
        sql/image_insert.sql
        sql/images_table.sql
        sql/imu_data_insert.sql
        sql/imu_data_table.sql
)

foreach (sql ${SQL_FILES})
    get_filename_component(name ${sql} NAME_WE)
    file(READ ${sql} contents)

    set(SQL_STRING_ENTRIES
            "${SQL_STRING_ENTRIES}\nconstexpr char const * const ${name} = R\"sql(\n${contents}\n)sql\";\n")
endforeach ()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sql_statements.hpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/sql_statements.hpp
        @ONLY

)


# Protobuf logic adopted from https://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers
# TODO(Jack): Should we compile database_proto as static like in the example linked above?
# TODO(Jack): Is there anyway to have the generated code go to the "generated/" folder? It would add some consistency
# with the generated sql code which we are otherwise lacking.
protobuf_generate_cpp(proto_sources proto_headers
        "proto/extracted_target.proto"
)
add_library(database_proto SHARED ${proto_sources} ${proto_headers})

set(SRC_FILES
        src/calibration_database.cpp
        src/sensor_data_interface.cpp
        src/serialization.cpp
        src/sqlite3_helpers.cpp
)
set(INCLUDE_DIRECTORIES
        ${CMAKE_BINARY_DIR}/database
        ${OpenCV_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        database_proto
        Eigen3::Eigen
        ${OpenCV_LIBS}
        protobuf::libprotobuf
        SQLite::SQLite3
        types
)
AddLibrary()

set(TESTS
        src/serialization.test.cpp
        src/sql.test.cpp
        src/sqlite3_helpers.test.cpp
        test/calibration_database.test.cpp
        test/sensor_data_interface.test.cpp
)
AddTests()