macro(AddLibrary)
    add_library(${LIBRARY_NAME} SHARED
            ${SRC_FILES}
    )
    target_include_directories(${LIBRARY_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
            $<INSTALL_INTERFACE:include>
    )
    target_include_directories(${LIBRARY_NAME} PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
            ${INCLUDE_DIRECTORIES}
    )
    target_link_libraries(${LIBRARY_NAME} PRIVATE
            ${LINK_LIBRARIES}
    )

    # Install headers default OFF - The entire library should only expose its functionality through the smallest
    # possible source code interface. At time of writing this is achieved by building all the functionality external
    # users could need into the application library. Therefore we only export the headers for the application package.
    # It would be that we also export headers for some types or other utility functions but that is it. If you want to
    # install the headers for a library just set set(INSTALL_HEADERS ON) in its cmakelists.
    if (NOT DEFINED INSTALL_HEADERS)
        set(INSTALL_HEADERS OFF)
    endif()

    # Install shared libs default ON - The application functions we export will depend on basically all the shared libs
    # we have. The one obvious exception here is any lib intended only for internal testing purposes (i.e.
    # testing_mocks/testing_utilities). Therefore this is default on, and if we do not want the library install than we
    # can call set(INSTALL_SO OFF) in its cmakelists.
    if (NOT DEFINED INSTALL_SO)
        set(INSTALL_SO ON)
    endif()

    if (INSTALL_HEADERS)
        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
                DESTINATION include/${PROJECT_NAME}
        )
    endif()

    if (INSTALL_SO)
        install(TARGETS ${LIBRARY_NAME}
                EXPORT reprojectionTargets
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
        )
    endif()

    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${LIBRARY_NAME} PRIVATE --coverage)
        target_link_options(${LIBRARY_NAME} PRIVATE --coverage)
    endif ()

endmacro()