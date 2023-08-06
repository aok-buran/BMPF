cmake_minimum_required(VERSION 3.5.1)
project(find_trajectory)

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
        ../find_path/find_path_core/include
        ../find_path/smart_path_finding/include
        ../core/robot/include
        ../core/monotone_cubic_interpolation/include
        ../core/robot/base/include
        ../core/collider/include
        ../core/collider/base/include
        ../core/solid3/include
        ../core/scene/include
        ../core/gl_scene/include
        ../core/urdf_reader/include
        ../core/misc/include
        ../core/scene/include/
        ../urdf_reader/include

        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

link_directories(
        monotone_cubic_interpolation
        smart_path_finding
        find_path_core
        collider
        urdf_reader
        scene
        misc
        gl_scene
        ${Boost_LIBRARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        src/monotone_trajectory_finder.cpp
        include/monotone_trajectory_finder.h
)

add_executable(testMonotoneTrajectoryFinder
        src/monotone_trajectory_finder.cpp
        include/monotone_trajectory_finder.h
        test/test_monotone_trajectory_finder.cpp
        )

target_link_libraries(testMonotoneTrajectoryFinder
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
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

add_executable(testMonotoneTrajectoryFinder2
        src/monotone_trajectory_finder.cpp
        include/monotone_trajectory_finder.h
        test/test_monotone_trajectory_finder_2.cpp
        )

target_link_libraries(testMonotoneTrajectoryFinder2
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        scene
        gl_scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        misc
        -lGL
        -lglut
        urdf_reader
        pthread
        tbbmalloc_proxy
        -ltbb
        tbbmalloc
        #  -fsanitize=address
        )


add_executable(DemoMonotoneTrajectoryFinder
        src/monotone_trajectory_finder.cpp
        include/monotone_trajectory_finder.h
        demo/demo_trajectory_finding.cpp
        )

target_link_libraries(DemoMonotoneTrajectoryFinder
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
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
        -lGL
        -lglut
        tbbmalloc_proxy
        -ltbb
        tbbmalloc
        #  -fsanitize=address
        )


add_executable(DemoMonotoneTrajectoryFinder2
        src/monotone_trajectory_finder.cpp
        include/monotone_trajectory_finder.h
        demo/demo_trajectory_finding_2.cpp
        )

target_link_libraries(DemoMonotoneTrajectoryFinder2
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
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


add_test(NAME TestMonotoneTrajectoryFinder COMMAND testMonotoneTrajectoryFinder)
add_test(NAME TestMonotoneTrajectoryFinder2 COMMAND testMonotoneTrajectoryFinder2)