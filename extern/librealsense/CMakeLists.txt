# Include the FetchContent module
include(FetchContent)

# Check if the librealsense target already exists
if (NOT TARGET realsense2)
  
  # Fetch librealsense from GitHub
  FetchContent_Declare(
    librealsense
    GIT_REPOSITORY https://github.com/IntelRealSense/librealsense.git
    GIT_TAG v2.54.2
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )

  FetchContent_GetProperties(librealsense)
  if(NOT librealsense_POPULATED)
    FetchContent_Populate(librealsense)
    # Set the librealsense options here
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
    set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(BUILD_GLSL_EXTENSIONS OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON_DOCS OFF CACHE BOOL "" FORCE)
    set(IMPORT_DEPTH_CAM_FW OFF CACHE BOOL "" FORCE)
    # Add more options as needed

    # Now add the subdirectory
    add_subdirectory(src EXCLUDE_FROM_ALL)
  endif()
  add_library(realsense2::realsense ALIAS realsense2)
else()
  message(STATUS "realsense2 is already a target. Skipping adding it twice")
endif()
