cmake_minimum_required(VERSION 3.5.1)
project(smart_path_finding)

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
        include
        ../find_path_core/include

        ../../core/robot/include
        ../../core/robot/base/include
        ../../core/collider/include
        ../../core/collider/base/include
        ../../core/solid3/include
        ../../core/scene/include
        ../../core/misc/include
        ../urdf_reader/include


        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

link_directories(
        find_path_core
        collider
        urdf_reader
        scene
        misc
        ${Boost_LIBRARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        src/multirobot_path_finder.cpp
        include/multirobot_path_finder.h
        src/continuous_path_finder.cpp
        include/continuous_path_finder.h
)

add_executable(testMultiRobotPathFinder2
        src/multirobot_path_finder.cpp
        include/multirobot_path_finder.h
        test/test_multirobot_path_finder_2.cpp
        src/continuous_path_finder.cpp
        include/continuous_path_finder.h
        )

target_link_libraries(testMultiRobotPathFinder2
        find_path_core
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        misc
        urdf_reader
        pthread
        tbbmalloc_proxy
        -ltbb
        tbbmalloc
        #  -fsanitize=address
        )

add_executable(testMultiRobotPathFinder
        src/multirobot_path_finder.cpp
        include/multirobot_path_finder.h
        test/test_multirobot_path_finder.cpp
        src/continuous_path_finder.cpp
        include/continuous_path_finder.h

        )

target_link_libraries(testMultiRobotPathFinder
        find_path_core
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        misc
        urdf_reader
        pthread
        tbbmalloc_proxy
        -ltbb
        tbbmalloc
        #  -fsanitize=address
        )


add_executable(testContinuousPathFinder2
        test/test_continuous_path_finder_2.cpp
        src/continuous_path_finder.cpp
        include/continuous_path_finder.h
        src/multirobot_path_finder.cpp
        include/multirobot_path_finder.h
        )

target_link_libraries(testContinuousPathFinder2
        find_path_core
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        misc
        urdf_reader
        pthread
        tbbmalloc_proxy
        -ltbb
        tbbmalloc
        #  -fsanitize=address
        )

add_executable(testContinuousPathFinder
        test/test_continuous_path_finder.cpp
        src/continuous_path_finder.cpp
        include/continuous_path_finder.h
        src/multirobot_path_finder.cpp
        include/multirobot_path_finder.h
        )

target_link_libraries(testContinuousPathFinder
        find_path_core
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        misc
        urdf_reader
        pthread
        tbbmalloc_proxy
        -ltbb
        tbbmalloc
        #  -fsanitize=address
        )


add_test(NAME testMultiRobotPathFinder COMMAND testMultiRobotPathFinder)
add_test(NAME testMultiRobotPathFinder2 COMMAND testMultiRobotPathFinder2)
add_test(NAME testContinuousPathFinder COMMAND testContinuousPathFinder)
add_test(NAME testContinuousPathFinder2 COMMAND testContinuousPathFinder2)