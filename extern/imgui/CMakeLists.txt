if (NOT TARGET imgui)
    set(imgui_SOURCE_DIR src)
    add_library(imgui STATIC
                ${imgui_SOURCE_DIR}/imgui.cpp
                ${imgui_SOURCE_DIR}/imgui_demo.cpp
                ${imgui_SOURCE_DIR}/imgui_draw.cpp
                ${imgui_SOURCE_DIR}/imgui_tables.cpp
                ${imgui_SOURCE_DIR}/imgui_widgets.cpp
                ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
                ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )

    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
        ${imgui_SOURCE_DIR}/examples/libs/gl3w
        ${imgui_SOURCE_DIR}/examples/libs/glfw/include
    )
else()
    message(STATUS "imgui is already a target. Skipping adding it twice")
endif()

add_library(imgui::imgui ALIAS imgui)