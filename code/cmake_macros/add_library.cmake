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

    # Install shared libs default ON
    if (NOT DEFINED INSTALL_SO)
        set(INSTALL_SO ON)
    endif()

    # Install headers default OFF
    if (NOT DEFINED INSTALL_HEADERS)
        set(INSTALL_HEADERS OFF)
    endif()

    if (INSTALL_SO)
        install(TARGETS ${LIBRARY_NAME}
                EXPORT reprojectionTargets
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
                RUNTIME DESTINATION bin
        )
    endif()

    if (INSTALL_HEADERS)
        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
                DESTINATION include
        )
    endif()


    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${LIBRARY_NAME} PRIVATE --coverage)
        target_link_options(${LIBRARY_NAME} PRIVATE --coverage)
    endif ()

endmacro()