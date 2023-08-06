cmake_minimum_required(VERSION  3.5.1)


PROJECT(FindPath CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${PROJECT_SOURCE_DIR}")

enable_testing()

add_subdirectory(solid3)
include_directories(solid3/include)

add_subdirectory(collider)
include_directories(collider/include)

add_subdirectory(urdf_reader)
include_directories(urdf_reader/include)

add_subdirectory(misc)
include_directories(misc/include)

add_subdirectory(robot)
include_directories(robot/include)

add_subdirectory(scene)
include_directories(scene/include)

add_subdirectory(gl_scene)
include_directories(gl_scene/include)

add_subdirectory(monotone_cubic_interpolation)
include_directories(monotone_cubic_interpolation/include)
