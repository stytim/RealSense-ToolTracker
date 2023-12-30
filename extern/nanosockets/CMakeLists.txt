if (NOT TARGET nanosockets)
    set(NANOSOCKETS_STATIC "1")

    add_subdirectory(src)

    set(NANOSOCKETS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(nanosockets INTERFACE ${NANOSOCKETS_SOURCE_DIR})

else()
    message(STATUS "nanosockets is already a target. Skipping adding it twice")
endif()

