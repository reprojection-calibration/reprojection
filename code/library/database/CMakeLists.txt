set(LIBRARY_NAME "database")

# NOTE(Jack): Because we want one single source of truth for both the library and python tooling we store the sql files
# under `code/sql`, outside of the cmake package. Therefore just like for the proto files below we need to backspace two
# directories "../../${sql}" to get to the actual file. If this is a bad design to hardcode reference paths outisde of
# the package we will find out one day in the future!
# WARN(Jack): If you change something in an .sql file you NEED to reload the cmake project! Otherwise the change will
# not be reflected in the generated header file.
set(SQL_FILES
        assets_delete.sql
        assets_insert.sql
        assets_select.sql
        assets_table.sql
        camera_info_insert.sql
        camera_info_select.sql
        camera_info_table.sql
        camera_poses_insert.sql
        camera_poses_select.sql
        camera_poses_table.sql
        control_points_insert.sql
        control_points_select.sql
        control_points_table.sql
        extracted_targets_insert.sql
        extracted_targets_select.sql
        extracted_targets_table.sql
        extrinsics_insert.sql
        extrinsics_select.sql
        extrinsics_table.sql
        gravity_insert.sql
        gravity_select.sql
        gravity_table.sql
        images_insert.sql
        images_select.sql
        images_table.sql
        imu_data_insert.sql
        imu_data_select.sql
        imu_data_table.sql
        intrinsics_insert.sql
        intrinsics_select.sql
        intrinsics_table.sql
        reprojection_errors_insert.sql
        reprojection_errors_table.sql
        spline_info_insert.sql
        spline_info_select.sql
        spline_info_table.sql
        steps_delete_trigger.sql
        steps_insert.sql
        steps_select.sql
        steps_table.sql
        steps_update_cache_key.sql
        target_info_insert.sql
        target_info_select.sql
        target_info_table.sql
        workflow_assets_insert.sql
        workflow_assets_table.sql
        workflow_steps_table.sql
        workflow_steps_upsert.sql
        workflows_insert.sql
        workflows_select.sql
        workflows_table.sql
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
        src/database_semantics.cpp
        src/serialization.cpp
        src/sqlite_helpers.cpp
        src/sqlite_exception.cpp
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
        config # Required because of hashing
        hashing # Only used to serialize the asset vector in workflow creation
        geometry # Required because of spline interface
        logging
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
        src/serialization.test.cpp
        src/toml_converters.test.cpp
        test/calibration_database.test.cpp
        test/sqlite_exception.test.cpp
)
AddTests()