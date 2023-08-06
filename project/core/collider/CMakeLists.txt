cmake_minimum_required(VERSION 3.5.1)
project(collider)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED Core)

cmake_policy(SET CMP0072 OLD)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(
        include
        ../solid3/include
        ../misc/include
        ${EIGEN3_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)

link_directories(
        solid3
)

add_library(
        ${PROJECT_NAME}
        src/solid_collider.cpp
        include/solid_collider.h
        src/base/solid_3d_object.cpp
        include/base/solid_3d_object.h
        src/base/stl_shape.cpp
        include/base/stl_shape.h
        src/solid_sync_collider.cpp
        include/solid_sync_collider.h
)


add_executable(testSolidCollision
        test/test_solid_collision.cpp
        include/solid_sync_collider.h
        src/solid_sync_collider.cpp
        src/solid_collider.cpp
        src/base/stl_shape.cpp
        src/base/solid_3d_object.cpp
        include/solid_collider.h
        include/base/collider.h
        include/base/stl_shape.h
        )


target_link_libraries(testSolidCollision
        urdf_reader
        pthread
        solid3
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        tbbmalloc_proxy
        tbbmalloc
        -lGL
        -lglut
        #-fsanitize=address
        )

add_test(NAME testSolidCollision COMMAND testSolidCollision)
