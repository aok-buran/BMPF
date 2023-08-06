cmake_minimum_required(VERSION 3.5.1)
project(scene)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

set(JSONCPP_LIBRARIES jsoncpp)


include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ../core/misc/include
        ../core/collider/include
        ../core/solid3/include
        ../core/urdf_reader/include
        ../collider/include
        ../collider/lumber/include
        ../core/robot/include
        ../core/robot/base/include
        include
)

link_directories(
        ${Boost_LIBRARY_DIR}
        urdf_reader
        collider
        robot
        solid3
        misc
)

add_library(
        ${PROJECT_NAME}
        src/scene.cpp
)


