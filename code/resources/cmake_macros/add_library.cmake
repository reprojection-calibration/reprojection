macro(AddLibrary)
    add_library(${LIBRARY_NAME} SHARED
            ${SRC_FILES}
    )
    target_include_directories(${LIBRARY_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )
    target_include_directories(${LIBRARY_NAME} PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
            ${INCLUDE_DIRECTORIES}
    )
    # NOTE(Jack): A simple rule is that if a package's header is found in a libraries public headers (ex. is found in a
    # header located in include/geometry for example), then that package needs to be added in the PUBLIC_LINK_LIBRARIES.
    # If the package's headers are only included in files found in src/ or test/ then that package needs to be added to
    # PRIVATE_LINK_LIBRARIES. This is a nice feature because the public libraries automatically propagate dependencies
    # which prevents us from having to include every library every time.
    target_link_libraries(${LIBRARY_NAME}
            PRIVATE
            ${PRIVATE_LINK_LIBRARIES}
            PUBLIC
            ${PUBLIC_LINK_LIBRARIES}
    )

    # We only want to expose the libraries functionality through the smallest possible source code interface, therefore
    # we set this to default OFF and only turn it on exactly when needed. As of time of writing (19.08.2026) we only
    # install the "application" and public "types" includes.
    if (NOT DEFINED REPROJECTION_INSTALL_HEADERS)
        set(REPROJECTION_INSTALL_HEADERS OFF)
    endif ()

    if (REPROJECTION_INSTALL_HEADERS)
        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
                DESTINATION include/${PROJECT_NAME}
        )
    endif ()

    # Because we build the project as a a bundle of shared libraries we need to install almost all of them (not
    # including test helpers), but only need to export a small subset that the consuming applications actually need to
    # know about. Just like the includes install right now we only export the "application" and public "types" targets.
    if (NOT DEFINED REPROJECTION_INSTALL_TARGET)
        set(REPROJECTION_INSTALL_TARGET ON)
    endif ()

    if (REPROJECTION_INSTALL_TARGET)
        # TODO(Jack): Use GNU install dirs here and for the includes.
        install(TARGETS ${LIBRARY_NAME}
                EXPORT reprojectionTargets
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
        )
    endif ()

    if (REPROJECTION_ENABLE_COVERAGE)
        target_compile_options(${LIBRARY_NAME} PRIVATE --coverage)
        target_link_options(${LIBRARY_NAME} PRIVATE --coverage)
    endif ()

endmacro()