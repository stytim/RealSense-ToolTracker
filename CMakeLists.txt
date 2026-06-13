cmake_minimum_required(VERSION 3.21)
project(ir-tracking)

# Set C++17 as the standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 -D_ENABLE_EXTENDED_ALIGNED_STORAGE=1")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE") # /PROFILE
    add_definitions("/MP")
    add_definitions("/MT")
else()
    # Warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

    # Remove Asio warning
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-variable")

    # Static library: -fPIC
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -march=native -fstack-protector")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
endif()

IF(MSVC)
   ADD_DEFINITIONS(/arch:AVX)
ENDIF(MSVC)

add_subdirectory(extern)

# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(realsense2_LIBRARY "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64\\realsense2.lib")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(realsense2_LIBRARY "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")                # TODO: Update this variable to correct path - folder to which your project will compile
    set(realsense2_INCLUDE_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found

else()
    # Find the RealSense SDK
    find_package(realsense2 REQUIRED)
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories for RealSense, OpenCV
include_directories(${realsense2_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

set(TRACKER_SOURCE_FILES
    include/IRStructs.h
    include/IRKalmanFilter.h
    include/IRToolTrack.h
    include/IRToolTracking.h
    src/IRToolTrack.cpp
    src/IRToolTracking.cpp 
)
include_directories(include)
# Add the IRToolTracking library
add_library(IRToolTracking STATIC ${TRACKER_SOURCE_FILES})
# Specify C++17 for IRToolTracking library
target_compile_features(IRToolTracking PUBLIC cxx_std_17)

# Link RealSense SDK and OpenCV with the IRToolTracking library
target_link_libraries(IRToolTracking PUBLIC Eigen3 ${realsense2_LIBRARY} opencv_core opencv_imgproc opencv_imgcodecs opencv_video)

################################################################################
# Dependencies for the main application



set(MACOS_ICON_NAME "app_icon.icns")
set(MACOS_ICON "${CMAKE_CURRENT_SOURCE_DIR}/resources/${MACOS_ICON_NAME}")


find_package(OpenGL REQUIRED)
include_directories(
    .
    ${OPENGL_INCLUDE_DIRS}
)

set(VIEWER_SOURCE_FILES
    win.rc
    include/ROMParser.h
    include/ViewerWindow.h
    include/Log.h
    src/ViewerWindow.cpp
    src/viewer_main.cpp
    src/Log.cpp
)

# On macOS the app icon is compiled into the .app bundle's Resources folder.
if(APPLE)
    list(APPEND VIEWER_SOURCE_FILES ${MACOS_ICON})
endif()

# Add the main application
add_executable(ir-tracking-app  ${VIEWER_SOURCE_FILES})

# -----------------------------------------------------------------------------
# Windowing / launch behavior: don't pop a separate console (CMD / Terminal)
# window when the app is launched from a GUI.
# -----------------------------------------------------------------------------
if(WIN32)
    # Build as a GUI-subsystem (/SUBSYSTEM:WINDOWS) executable so launching from
    # Explorer or a Start-menu shortcut no longer spawns a CMD window. Keep the
    # standard int main(...) entry point instead of WinMain by pointing the
    # linker at the console CRT startup, which calls main(). The app still
    # AttachConsole()s to a parent terminal at runtime so `ir-tracking-app.exe`
    # launched from cmd/PowerShell keeps printing logs.
    set_target_properties(ir-tracking-app PROPERTIES WIN32_EXECUTABLE TRUE)
    if(MSVC)
        target_link_options(ir-tracking-app PRIVATE /ENTRY:mainCRTStartup)
    endif()
endif()

if(APPLE)
    # Build a .app bundle so double-clicking it in Finder does not open a
    # Terminal window (a bare Unix executable would). Logs still go to the
    # terminal when the inner binary is run from a shell, and always to the
    # in-app log console.
    set_source_files_properties(${MACOS_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
    set_target_properties(ir-tracking-app PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/resources/Info.plist.in"
        MACOSX_BUNDLE_BUNDLE_NAME "IR Tracking App"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.medivis.ir-tracking-app"
        MACOSX_BUNDLE_ICON_FILE "${MACOS_ICON_NAME}"
        MACOSX_BUNDLE_BUNDLE_VERSION "0.0.0"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "0.0.0"
    )
endif()

include_directories( ${IMGUI_DIR} )
include_directories( ${IMGUI_DIR}/backends)
include_directories( ${OPENGL_INCLUDE_DIR} )
include_directories( ${GLFW_INCLUDE_DIR} )

# Specify C++17 for the main application
target_compile_features(ir-tracking-app PUBLIC cxx_std_17)

target_link_libraries(ir-tracking-app PRIVATE
                        IRToolTracking
                        glfw::glfw
                        imgui::imgui
                        ${OPENGL_LIBRARIES} 
                        nanosockets
                        nfd
                        json)





