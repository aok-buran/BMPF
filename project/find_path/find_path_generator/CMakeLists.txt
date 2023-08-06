cmake_minimum_required(VERSION 3.5.1)
project(find_path_generator)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)
set(JSONCPP_LIBRARIES jsoncpp)
cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

#install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
        include
        ../../core/robot/include
        ../../core/robot/base/include
        ../../core/scene/include
        ../../core/misc/include
        ../../core/urdf_reader/include
        ../../core/solid3/include
        ../../core/collider/include
        ../find_path_core/include
        ../smart_path_finding/include
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
        ${Boost_LIBRARY_DIR}
        urdf_reader
        gl_scene
        robot
        solid3
        collider
        misc
        smart_path_finding
        find_path_core
)


add_library(
        ${PROJECT_NAME}
        include/generator.h
        src/generator.cpp
)


add_executable(Exp1 include/generator.h src/generator.cpp src/exp_1.cpp)


target_link_libraries(Exp1
        find_path_core
        smart_path_finding
        scene
        robot
        collider
        urdf_reader
        pthread
        misc
        gl_scene
        solid3
        ${CMAKE_THREAD_LIBS_INIT}
        ${JSONCPP_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        -lboost_filesystem
        -lboost_system
        -lGL
        -lglut
        -fsanitize=address
        )

add_executable(Exp2 include/generator.h src/generator.cpp src/exp_2.cpp)


target_link_libraries(Exp2
        smart_path_finding
        find_path_core
        scene
        robot
        collider
        urdf_reader
        pthread
        misc
        gl_scene
        solid3
        ${CMAKE_THREAD_LIBS_INIT}
        ${JSONCPP_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        -lboost_filesystem
        -lboost_system
        -lGL
        -lglut
        -fsanitize=address
        )

