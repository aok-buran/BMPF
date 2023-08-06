cmake_minimum_required(VERSION  3.5.1)


PROJECT(FindPath CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${PROJECT_SOURCE_DIR}")

set( CMAKE_BUILD_TYPE Debug )


enable_testing()




add_compile_options(
        -O3
        -g3
        #-Wall
        )


add_subdirectory(project/core)

add_subdirectory(project/scene_editor)

add_subdirectory(project/find_path)

add_subdirectory(project/render_path)

add_subdirectory(project/optimize_path)

add_subdirectory(project/find_trajectory)

add_subdirectory(project/network)

