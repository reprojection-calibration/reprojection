set(LIBRARY_NAME "demos")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(SRC_FILES
        src/image_source.cpp
)
set(LINK_LIBRARIES
        calibration
        calibration_data_views
        database
        Eigen3::Eigen
        eigen_utilities
        feature_extraction
        geometry
        testing_mocks
        testing_utilities
        types
        ${apriltag_LIBRARIES}
        optimization
        ${OpenCV_LIBS}
        pnp
        projection_functions
)
AddLibrary()

set(TESTS
        test/image_source.test.cpp
)
AddTests()

set(EXAMPLES
        examples/feature_extraction.cpp
        examples/pose_initialization.cpp
        examples/testing_mock_db_generation.cpp
)
foreach (EXAMPLE IN LISTS EXAMPLES)
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WLE)
    set(EXAMPLE_NAME ${LIBRARY_NAME}.${EXAMPLE_NAME})

    add_executable(${EXAMPLE_NAME} ${EXAMPLE})
    target_link_libraries(${EXAMPLE_NAME}
            ${LIBRARY_NAME}
            ${LINK_LIBRARIES}
    )
endforeach ()