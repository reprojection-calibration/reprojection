set(LIBRARY_NAME "projection_functions")

set(SRC_FILES
        src/double_sphere.cpp
        src/initialize_camera.cpp
        src/pinhole.cpp
        src/pinhole_radtan4.cpp
        src/unified_camera_model.cpp
)
set(PRIVATE_LINK_LIBRARIES
        types_internal
)
set(PUBLIC_LINK_LIBRARIES
        Ceres::ceres
)
AddLibrary()

set(TEST_LINK_LIBRARIES
        testing_utilities
)
set(TESTS
        test/camera_model.test.cpp
        test/double_sphere.test.cpp
        test/image_bounds.test.cpp
        test/initialize_camera.test.cpp
        test/pinhole.test.cpp
        test/pinhole_radtan4.test.cpp
        test/unified_camera_model.test.cpp
)
AddTests()

# NOTE(Jack): We only support the bindings on ubuntu 24.04. We only need them to work with the dashboard image and there
# were problems with the source install version of things and the pybind library which is why we limit this. Is that
# still a problem?

if (NOT DEFINED REPROJECTION_BUILD_PYTHON_BINDINGS)
    set(REPROJECTION_BUILD_PYTHON_BINDINGS OFF)
endif ()

if (REPROJECTION_BUILD_PYTHON_BINDINGS)
    # TODO(Jack): CLion docker/cmake toolchains do not allow us to evaluate variables which means that if we wanted to
    # pass pybind11_DIR via the regular -D... cmake method every developer would need to have the massive absolute path
    # to pybind in the venv. Therefore to avoid this we set PYBIND11_DIR as environmental variable in the
    # development-stage (which unfortunately we have to hardcode).
    if(DEFINED ENV{PYBIND11_DIR})
        set(pybind11_DIR "$ENV{PYBIND11_DIR}")
    endif()

    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 CONFIG REQUIRED)

    # TODO(Jack): This current install process will install the entire library. I would prefer it would only install the
    # part actually needed by the projection_functions. Maybe we can use components to solve this?
    pybind11_add_module(projection_function_python_binding src/python_bindings.cpp)
    target_link_libraries(projection_function_python_binding
            PRIVATE
            ${LIBRARY_NAME}
            ${PRIVATE_LINK_LIBRARIES}
    )

    # This suppresses an elevated warning from pybind11/Eigen
    target_compile_options(
            projection_function_python_binding
            PRIVATE
            -Wno-maybe-uninitialized
    )

    set_target_properties(projection_function_python_binding
            PROPERTIES
            INSTALL_RPATH "$ORIGIN/lib"
    )

    # TODO(Jack): What is the proper install destination? Does this work ?
    install(
            TARGETS
            projection_function_python_binding
            DESTINATION
            .
    )

    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(
            NAME
            projection_function_python_binding_unit_test
            COMMAND
            ${Python3_EXECUTABLE} -m unittest discover --start-directory ${CMAKE_CURRENT_SOURCE_DIR}/src --verbose
    )
endif ()
