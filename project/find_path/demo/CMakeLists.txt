cmake_minimum_required(VERSION 3.5.1)
project(DemoFindPath)

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

include_directories(
        include
        ../client/include
        ../find_path/find_path_core/include/
        ../../core/misc/include
        ../../core/gl_scene/include
        ../../core/urdf_reader/include
        ../../core/collider/include
        ../../core/solid3/include
        ../../core/collider/base/include
        ../../core/robot/include
        ../../core/robot/base/include
        ../../core/scene/include
        ../../solid3/include
        ../../scene_editor/include
        ../client/include
        ${EIGEN3_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)


link_directories(
        ${Boost_LIBRARY_DIR}
        find_path_core
        smart_path_finding
        urdf_reader
        gl_scene
        robot
        solid3
        collider
        misc
)


add_executable(DemoURDFRobot
        src/demo_urdf_robot.cpp
        )

target_link_libraries(DemoURDFRobot
        robot
        collider
        urdf_reader
        pthread
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


add_executable(DemoDHRobot
        src/demo_dh_robot.cpp
        )

target_link_libraries(DemoDHRobot
        robot
        collider
        urdf_reader
        pthread
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


add_executable(DemoPathFinding2 src/demo_path_finding_2.cpp)

target_link_libraries(DemoPathFinding2
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

add_executable(DemoMRPathFinding2 src/demo_mr_path_finding_2.cpp)

target_link_libraries(DemoMRPathFinding2
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

add_executable(DemoMRPathFinding src/demo_mr_path_finding.cpp)

target_link_libraries(DemoMRPathFinding
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


add_executable(DemoPathFinding src/demo_path_finding.cpp)

target_link_libraries(DemoPathFinding
        smart_path_finding
        find_path_core
        scene
        robot
        collider
        urdf_reader
        pthread
        gl_scene
        misc
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


add_executable(DemoCPathFinding src/demo_continuous_path_finding.cpp)

target_link_libraries(DemoCPathFinding
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



add_executable(DemoCPathFinding2 src/demo_continuous_path_finding_2.cpp)

target_link_libraries(DemoCPathFinding2
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

