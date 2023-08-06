cmake_minimum_required(VERSION 3.5.1)
project(find_path_core)

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
        ../../core/collider/include
        ../../core/collider/base/include
        ../../core/solid3/include
        ../../core/scene/include
        ../../core/misc/include
        ../../core/urdf_reader/include
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        /usr/include/jsoncpp
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

link_directories(
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
        src/one_direction_path_finder.cpp
        include/one_direction_path_finder.h

        src/all_directions_path_finder.cpp
        include/all_directions_path_finder.h

        src/one_direction_ordered_path_finder.cpp
        include/one_direction_ordered_path_finder.h

        src/one_direction_sync_path_finder.cpp
        include/one_direction_sync_path_finder.h

        src/base/path_finder.cpp
        include/base/path_finder.h

        include/base/path_node.h

        src/base/grid_path_finder.cpp
        include/base/grid_path_finder.h

        src/base/node_grid_path_finder.cpp
        include/base/node_grid_path_finder.h

)


add_executable(DemoFreePointFinding
        include/one_direction_path_finder.h
        src/one_direction_path_finder.cpp
        src/one_direction_ordered_path_finder.cpp
        include/one_direction_ordered_path_finder.h
        src/base/path_finder.cpp
        src/base/grid_path_finder.cpp
        src/base/node_grid_path_finder.cpp
        include/base/node_grid_path_finder.h
        demo/free_point_finding.cpp
        )


target_link_libraries(DemoFreePointFinding
        misc
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
        -ltbb
        -fsanitize=address
        )


add_executable(testOneDirectionPathFinder
        test/test_one_direction_path_finder.cpp
        include/one_direction_path_finder.h
        src/one_direction_path_finder.cpp
        src/base/path_finder.cpp
        src/base/grid_path_finder.cpp
        src/base/node_grid_path_finder.cpp
        include/base/node_grid_path_finder.h
        )

target_link_libraries(testOneDirectionPathFinder
        misc
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
        -ltbb
        -lboost_filesystem
        -lboost_system
        -lGL
        -lglut
        #-fsanitize=address
        )

add_executable(testAllDirectionPathFinder
        test/test_all_directions_path_finder.cpp
        include/all_directions_path_finder.h
        src/all_directions_path_finder.cpp
        include/one_direction_path_finder.h
        src/one_direction_path_finder.cpp
        src/base/path_finder.cpp
        src/base/grid_path_finder.cpp
        src/base/node_grid_path_finder.cpp
        include/base/node_grid_path_finder.h
        )

target_link_libraries(testAllDirectionPathFinder
        misc
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        tbbmalloc_proxy
        tbbmalloc
        -ltbb
        -lboost_filesystem
        -lboost_system
        -lGL
        -lglut
        #-fsanitize=address
        )


add_executable(testOneDirectionOrderedPathFinder
        src/base/path_finder.cpp
        src/base/grid_path_finder.cpp
        test/test_one_direction_ordered_path_finder.cpp
        src/one_direction_path_finder.cpp
        include/one_direction_path_finder.h
        src/one_direction_ordered_path_finder.cpp
        include/one_direction_ordered_path_finder.h
        src/base/node_grid_path_finder.cpp
        include/base/node_grid_path_finder.h
        )

target_link_libraries(testOneDirectionOrderedPathFinder
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


add_executable(testOneDirectionSyncPathFinder
        test/test_one_direction_sync_path_finder.cpp
        include/one_direction_sync_path_finder.h
        src/one_direction_sync_path_finder.cpp
        include/one_direction_path_finder.h
        src/one_direction_path_finder.cpp
        src/base/path_finder.cpp
        src/base/grid_path_finder.cpp
        src/base/node_grid_path_finder.cpp
        include/base/node_grid_path_finder.h
        )

target_link_libraries(testOneDirectionSyncPathFinder
        scene
        robot
        collider
        ${JSONCPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        solid3
        urdf_reader
        pthread
        misc
        tbbmalloc_proxy
        tbbmalloc
        -ltbb
        -fsanitize=address
        )


add_test(NAME testAllDirectionPathFinder COMMAND testAllDirectionPathFinder)
add_test(NAME testOneDirectionPathFinder COMMAND testOneDirectionPathFinder)
add_test(NAME testOneDirectionOrderedPathFinder COMMAND testOneDirectionOrderedPathFinder)
add_test(NAME testOneDirectionSyncPathFinder COMMAND testOneDirectionSyncPathFinder)





