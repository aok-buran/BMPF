cmake_minimum_required(VERSION 3.5.1)
project(robot)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)
find_package(jsoncpp REQUIRED)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS filesystem)

set(JSONCPP_LIBRARIES jsoncpp)

include_directories(
        /usr/include/jsoncpp
        base/include
        include
        ../misc/include
        ../gl_scene/include
        ../urdf_reader/include

        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}


)

link_directories(
        ${Boost_LIBRARY_DIR}
        urdf_reader
        gl_scene
        misc
)

add_library(
        ${PROJECT_NAME}
        src/urdf_robot.cpp
        include/urdf_robot.h
        src/dh_robot.cpp
        include/dh_robot.h
        src/base/robot.cpp
        include/base/robot.h
        src/base/joint_params.cpp
        include/base/joint_params.h
        src/base/link.cpp
        include/base/link.h
        include/base/joint.h
)




add_executable(testURDFRobot
        test/test_urdf_robot.cpp
        include/urdf_robot.h
        src/urdf_robot.cpp
        src/base/robot.cpp
        include/base/robot.h
        src/base/joint_params.cpp
        include/base/joint_params.h
        src/base/link.cpp
        include/base/link.h
        include/base/joint.h)

target_link_libraries(testURDFRobot
        misc
        urdf_reader
        ${Boost_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}

        tbbmalloc_proxy
        tbbmalloc

     #   -fsanitize=address
        -lboost_filesystem
        -lboost_system
        )



add_executable(testDHRobot
        test/test_dh.cpp
        include/dh_robot.h
        src/dh_robot.cpp

        include/urdf_robot.h
        src/urdf_robot.cpp


        src/base/robot.cpp
        include/base/robot.h
        src/base/joint_params.cpp
        include/base/joint_params.h
        src/base/link.cpp
        include/base/link.h
        include/base/joint.h
        )

target_link_libraries(testDHRobot
        ${JSONCPP_LIBRARIES}
        urdf_reader
        misc
        ${Boost_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        tbbmalloc_proxy
        tbbmalloc
       # -fsanitize=address
        -lboost_filesystem
        -lboost_system
        )



add_executable(testIO test/test_io.cpp)

add_test(NAME testIO COMMAND testIO)
add_test(NAME testURDFRobot COMMAND testURDFRobot)
add_test(NAME testDHRobot COMMAND testDHRobot)
