macro(AddLibrary)
    add_library(${LIBRARY_NAME} SHARED
            ${SRC_FILES}
    )
    target_include_directories(${LIBRARY_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    )
    target_include_directories(${LIBRARY_NAME} PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
            ${INCLUDE_DIRECTORIES}
    )
    target_link_libraries(${LIBRARY_NAME} PRIVATE
            ${LINK_LIBRARIES}
    )

    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${LIBRARY_NAME} PRIVATE --coverage)
        target_link_options(${LIBRARY_NAME} PRIVATE --coverage)
    endif ()

endmacro()