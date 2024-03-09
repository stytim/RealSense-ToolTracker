if (NOT TARGET json)

    if (POLICY CMP0077)
        # Allow CMake 3.13+ to override options when using FetchContent / add_subdirectory.
        cmake_policy(SET CMP0077 NEW)
    endif ()

    option(JSON_Diagnostics         "Use extended diagnostic messages." OFF)
    option(JSON_ImplicitConversions "Enable implicit conversions." OFF)
    option(JSON_SystemInclude       "Include as system headers (skip for clang-tidy)." OFF)

    ##
    ## CONFIGURATION
    ##

    set(NLOHMANN_JSON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

    if (JSON_ImplicitConversions)
        message(WARNING "JSON: Implicit conversions are enabled")
    endif()

    if (JSON_Diagnostics)
        message(STATUS "JSON: Diagnostics enabled")
    endif()

    if (JSON_SystemInclude)
        set(NLOHMANN_JSON_SYSTEM_INCLUDE "SYSTEM")
    endif()

    ##
    ## TARGET
    ## create target and add include path
    ##
    add_library(json INTERFACE)
    add_library(json::json ALIAS json)

    target_compile_features(json INTERFACE cxx_std_17)

    target_compile_definitions(
        json
        INTERFACE
        $<$<NOT:$<BOOL:${JSON_ImplicitConversions}>>:JSON_USE_IMPLICIT_CONVERSIONS=0>
        $<$<BOOL:${JSON_Diagnostics}>:JSON_DIAGNOSTICS=1>
    )

    target_include_directories(
        json
        ${NLOHMANN_JSON_SYSTEM_INCLUDE} INTERFACE
        "${NLOHMANN_JSON_DIR}"
    )
else()
    message(STATUS "json is already a target. Skipping adding it twice")
endif()