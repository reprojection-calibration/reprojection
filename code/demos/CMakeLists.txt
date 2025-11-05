set(LIBRARY_NAME "demos")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(LINK_LIBRARIES
        Eigen3::Eigen
        eigen_utilities
        feature_extraction
        types
        ${apriltag_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
)


set(EXAMPLES
        examples/feature_extraction.cpp
)
foreach (EXAMPLE IN LISTS EXAMPLES)
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WLE)
    set(EXAMPLE_NAME ${LIBRARY_NAME}.${EXAMPLE_NAME})

    add_executable(${EXAMPLE_NAME} ${EXAMPLE})
    target_link_libraries(${EXAMPLE_NAME}
            ${LINK_LIBRARIES}
    )
endforeach ()
