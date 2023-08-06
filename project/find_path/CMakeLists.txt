cmake_minimum_required(VERSION  3.5.1)


PROJECT(FindPath CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${PROJECT_SOURCE_DIR}")

enable_testing()


add_subdirectory(find_path_core)
include_directories(find_path_core/include)


add_subdirectory(smart_path_finding)
include_directories(smart_path_finding/include)


add_subdirectory(find_path_generator)
include_directories(find_path_generator/include)

add_subdirectory(demo)
include_directories(demo/include)