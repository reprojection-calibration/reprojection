macro(AddExamples)
    foreach (EXAMPLE IN LISTS EXAMPLES)
        get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WLE)
        set(EXAMPLE_NAME ${LIBRARY_NAME}.${EXAMPLE_NAME})

        add_executable(${EXAMPLE_NAME} ${EXAMPLE})
        target_include_directories(${EXAMPLE_NAME} PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
                ${INCLUDE_DIRECTORIES}
        )
        target_link_libraries(${EXAMPLE_NAME}
                ${LIBRARY_NAME}
                ${LINK_LIBRARIES}
        )

        if (NOT DEFINED INSTALL_EXAMPLES)
            set(INSTALL_EXAMPLES OFF)
        endif()

        if (INSTALL_EXAMPLES)
            install(TARGETS ${EXAMPLE_NAME}
                    EXPORT reprojectionTargets
                    RUNTIME DESTINATION bin
            )
        endif()
    endforeach ()
endmacro()