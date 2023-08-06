cmake_minimum_required(VERSION  3.5.1)


PROJECT(optimize_path CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${PROJECT_SOURCE_DIR}")

add_compile_options(-std=c++14)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS filesystem)

set(JSONCPP_LIBRARIES jsoncpp)

#install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
        include
        ../core/robot/include
        ../core/robot/base/include
        ../core/scene/include
        ../core/misc/include
        ../core/urdf_reader/include
        ../core/solid3/include
        ../core/collider/include
        ../find_path/find_path_core/include
        ../find_path/smart_path_finding/include
        ../find_path/find_path_generator/include
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
        find_path_generator
        scene
        robot
        collider
        solid3
        urdf_reader
        misc
        ${Boost_LIBRARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        src/optimize_path.cpp
        include/optimize_path.h
        src/optimize_path_median.cpp
        include/optimize_path_median.h
        src/optimize_path_newton.cpp
        include/optimize_path_newton.h
        src/optimize_generator.cpp
        include/optimize_generator.h
)

target_link_libraries(
        ${PROJECT_NAME}
        find_path_generator
        smart_path_finding
        find_path_core
)


add_executable(OptimizeExp1
        src/optimize_path.cpp
        include/optimize_path.h
        src/optimize_path_median.cpp
        include/optimize_path_median.h
        src/optimize_path_newton.cpp
        include/optimize_path_newton.h
        src/optimize_generator.cpp
        include/optimize_generator.h
        src/exp_1.cpp
        )


target_link_libraries(OptimizeExp1
        find_path_generator
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

add_executable(OptimizeExp2
        src/optimize_path.cpp
        include/optimize_path.h
        src/optimize_path_median.cpp
        include/optimize_path_median.h
        src/optimize_path_newton.cpp
        include/optimize_path_newton.h
        src/optimize_generator.cpp
        include/optimize_generator.h
        src/exp_2.cpp
        )


target_link_libraries(OptimizeExp2
        find_path_generator
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


