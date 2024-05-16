cmake_minimum_required (VERSION 3.29)

project(
    mc_traj_test
    VERSION 1.0.0
    DESCRIPTION "A Simple ImGui project"
    HOMEPAGE_URL "kindler.xyz"
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options("-Wall" "-Wextra")
    add_compile_options("-Wno-missing-field-initializers")

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options("-g" "-Og")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options("-g" "-O2")
    endif()
elseif(MSVC)
    # Enable edit & continue
    #
    add_compile_options("/ZI")
    add_link_options("/INCREMENTAL")
endif()


# Main executable
#
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mc_traj.c
    src/traj_window.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC src)

target_link_libraries(${PROJECT_NAME}
    imgui
    implot
)


# Add libraries
#
add_definitions(-DIMGUI_USER_CONFIG=\"${PROJECT_SOURCE_DIR}/src/imconfig.h\")
add_subdirectory(lib)


# Add platform specific libraries
#
if(WIN32)
    target_link_libraries(${PROJECT_NAME} opengl32 gdi32)
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} GL dl pthread udev)
endif()


