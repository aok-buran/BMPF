cmake_minimum_required(VERSION 3.5.1)
project(gl_scene)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Widgets CONFIG REQUIRED)


set(JSONCPP_LIBRARIES jsoncpp)


include_directories(
        include
        ../find_path/find_path_generator/include
        ../find_path/find_path_core/include/
        ../core/misc/include
        ../urdf_reader/include
        ../core/collider/include
        ../core/solid3/include
        ../core/collider/base/include
        ../core/robot/include
        ../core/robot/base/include
        ../core/scene/include
        ../solid3/include
        ../scene_editor/include
        ../core/gl_scene/include
        ${EIGEN3_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)

link_directories(
        ${Boost_LIBRARY_DIR}
        scene
        robot
        find_path_core
        collider
        misc
        urdf_reader
        pthread
        solid3
        find_path_generator
)


add_library(
        ${PROJECT_NAME}
        src/camera.cpp
        include/camera.h
        src/gl_scene.cpp
        include/gl_scene.h
        src/qt_gl_drawer.cpp
        include/qt_gl_drawer.h
        src/qt_gl_widget.cpp
        include/qt_gl_widget.h
)


target_link_libraries(
        ${PROJECT_NAME}
        Qt5::Widgets
        )