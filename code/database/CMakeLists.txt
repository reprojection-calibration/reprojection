set(LIBRARY_NAME "database")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(SQLite3 REQUIRED)

# Protobuf logic adopted from https://stackoverflow.com/questions/20824194/cmake-with-google-protocol-buffers
# TODO(Jack): Should we compile database_proto as static like in the example?
protobuf_generate_cpp(ProtoSources ProtoHeaders
        "proto/extracted_target.proto"
)
add_library(database_proto SHARED ${ProtoSources} ${ProtoHeaders})

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
        ${PROTOBUF_LIBRARY}
        SQLite::SQLite3
        types
)
AddLibrary()

set(TESTS
        src/sql.test.cpp
        src/sqlite3_helpers.test.cpp
        test/calibration_database.test.cpp
        test/sensor_data_interface.test.cpp
)
AddTests()