# SDL2
#
find_package(SDL2 REQUIRED)

# Dear ImGui
#
add_library(imgui
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
#    imgui/backends/imgui_impl_opengl3.cpp
    imgui_impl_opengl3_srgb.cpp
    imgui/backends/imgui_impl_sdl2.cpp
    imgui/misc/cpp/imgui_stdlib.cpp
)

target_include_directories(imgui
    PUBLIC
    imgui
    imgui/backends
    imgui/examples
    imgui/misc/cpp
)

target_link_libraries(imgui SDL2::SDL2 SDL2::SDL2main)


# ImPlot
#
add_library(implot
    implot/implot.cpp
    implot/implot_demo.cpp
    implot/implot_items.cpp
)

target_include_directories(implot
    PUBLIC
    implot
)

target_link_libraries(implot imgui)

target_compile_definitions(implot
    INTERFACE
    IMPLOT_DISABLE_OBSOLETE_FUNCTIONS
)
