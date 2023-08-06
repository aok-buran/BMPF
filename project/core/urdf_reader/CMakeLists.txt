cmake_minimum_required(VERSION 3.5.1)

project(urdf_reader)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED thread system log)
set(Boost_USE_STATIC_LIBS ON)
add_definitions(-DBOOST_LOG_DYN_LINK)

find_package(urdfdom_headers REQUIRED)
find_package(urdfdom REQUIRED)
find_package(TinyXML REQUIRED)

# Find version components
if(NOT urdfdom_headers_VERSION)
    set(urdfdom_headers_VERSION "0.0.0")
endif()

string(REGEX REPLACE "^([0-9]+).*" "\\1" URDFDOM_HEADERS_MAJOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_MINOR_VERSION "${urdfdom_headers_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" URDFDOM_HEADERS_REVISION_VERSION "${urdfdom_headers_VERSION}")

#install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    #  ${urdfdom_INCLUDE_DIRS}
    #  ${urdfdom_headers_INCLUDE_DIRS}
    ../urdfdom/urdf_parser/include
    ${TinyXML_INCLUDE_DIRS}
    )

link_directories(
    ${Boost_LIBRARY_DIRS}
    )

add_library(
    ${PROJECT_NAME}
    src/model.cpp
        ../misc/include/matrix_math.h)


set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(
    ${PROJECT_NAME}
    ${TinyXML_LIBRARIES}
    #    ${urdfdom_LIBRARIES}
    urdfdom_model
    urdfdom_world
    urdfdom_sensor
    urdfdom_model_state
    ${Boost_SYSTEM_LIBRARY}
    boost_log
    )

message("Boost_SYSTEM_LIBRARY ${Boost_SYSTEM_LIBRARY}")

#install(TARGETS ${PROJECT_NAME}
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#

