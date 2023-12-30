if (NOT TARGET Eigen3)
    set(Eigen3_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
    # Set the Eigen project options
    option(EIGEN_BUILD_DOC "" OFF)
    option(BUILD_TESTING "" OFF)
    option(EIGEN_SPLIT_LARGE_TESTS "" OFF)
    option(EIGEN_BUILD_PKGCONFIG "" OFF)

    add_subdirectory(src)

    # Eigen doesn't define an install target, so we define an interface library
    add_library(Eigen3 INTERFACE)
    target_include_directories(Eigen3 INTERFACE ${Eigen3_SOURCE_DIR})
else()
    message(STATUS "Eigen3 is already a target. Skipping adding it twice")
endif()

