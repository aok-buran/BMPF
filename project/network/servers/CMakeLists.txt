cmake_minimum_required(VERSION 3.5.1)
project(network_servers)

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
        ../../find_path/find_path_core/include
        ../../find_path/smart_path_finding/include
        ../../core/robot/include
        ../../core/monotone_cubic_interpolation/include
        ../../core/robot/base/include
        ../../core/collider/include
        ../../optimize_path/include
        ../../core/collider/base/include
        ../../core/solid3/include
        ../../core/scene/include/
        ../../core/gl_scene/include
        ../../core/urdf_reader/include
        ../../core/misc/include
        ../../core/scene/include/
        ../../urdf_reader/include
        ../../find_trajectory/include/
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
        optimize_path
        ${Boost_LIBRARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        src/base_server.cpp
        include/base_server.h
        include/path_finding_server.h
        src/path_finding_server.cpp
        include/multirobot_pf_server.h
        include/continuous_pf_server.h
        include/path_optimising_server.h
        src/path_optimising_server.cpp
        src/trajectory_finding_server.cpp
        include/trajectory_finding_server.h
)



add_executable(DemoMultirobotPFServer
        demo/demo_multirobot_pf_server.cpp
        src/base_server.cpp
        include/base_server.h
        include/path_finding_server.h
        include/multirobot_pf_server.h
        src/path_finding_server.cpp
        )

target_link_libraries(DemoMultirobotPFServer
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


add_executable(DemoContinuousPFServer
        demo/demo_continuous_pf_server.cpp
        src/base_server.cpp
        include/base_server.h
        include/path_finding_server.h
        include/continuous_pf_server.h
        src/path_finding_server.cpp
        )

target_link_libraries(DemoContinuousPFServer
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



add_executable(DemoPathOptimisingServer
        demo/demo_path_optimizing_server.cpp
        src/base_server.cpp
        include/base_server.h
        src/path_finding_server.cpp
        include/path_optimising_server.h
        src/path_optimising_server.cpp
        )

target_link_libraries(DemoPathOptimisingServer
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
        scene
        robot
        optimize_path
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



add_executable(DemoTrajectoryFindingServer
        demo/demo_trajectory_finding_server.cpp
        src/base_server.cpp
        include/base_server.h
        src/trajectory_finding_server.cpp
        include/trajectory_finding_server.h
        )

target_link_libraries(DemoTrajectoryFindingServer
        find_trajectory
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
        scene
        robot
        optimize_path
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




add_executable(ConsoleMultirobotPFServer
        console/console_multirobot_pf_server.cpp
        src/base_server.cpp
        include/base_server.h
        include/path_finding_server.h
        include/multirobot_pf_server.h
        src/path_finding_server.cpp
        )

target_link_libraries(ConsoleMultirobotPFServer
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


add_executable(ConsoleContinuousPFServer
        console/console_continuous_pf_server.cpp
        src/base_server.cpp
        include/base_server.h
        include/path_finding_server.h
        include/continuous_pf_server.h
        src/path_finding_server.cpp
        )

target_link_libraries(ConsoleContinuousPFServer
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



add_executable(ConsolePathOptimisingServer
        console/console_path_optimizing_server.cpp
        src/base_server.cpp
        include/base_server.h
        src/path_finding_server.cpp
        include/path_optimising_server.h
        src/path_optimising_server.cpp
        )

target_link_libraries(ConsolePathOptimisingServer
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
        scene
        robot
        optimize_path
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



add_executable(ConsoleTrajectoryFindingServer
        console/console_trajectory_finding_server.cpp
        src/base_server.cpp
        include/base_server.h
        src/trajectory_finding_server.cpp
        include/trajectory_finding_server.h
        )

target_link_libraries(ConsoleTrajectoryFindingServer
        find_trajectory
        smart_path_finding
        monotone_cubic_interpolation
        find_path_core
        gl_scene
        scene
        robot
        optimize_path
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

