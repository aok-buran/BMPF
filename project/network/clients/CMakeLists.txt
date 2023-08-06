cmake_minimum_required(VERSION 3.5.1)
project(network_clients)

add_compile_options(-std=c++14)

find_package(jsoncpp REQUIRED)
set(JSONCPP_LIBRARIES jsoncpp)

#install(FILES ${generated_compat_header} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
        include
        /usr/include/jsoncpp
)

#
#add_library(
#        ${PROJECT_NAME}
#        src/monotone_trajectory_finder.cpp
#        include/monotone_trajectory_finder.h
#)



add_executable(ClientPF
        demo/demo_path_finding_client.cpp
        include/base_client.h
        src/base_client.cpp
        include/path_finding_client.h
        src/path_finding_client.cpp
        )

target_link_libraries(ClientPF
        ${JSONCPP_LIBRARIES}
        )




add_executable(ClientPF2
        demo/demo_path_finding_client_2.cpp
        include/base_client.h
        src/base_client.cpp
        include/path_finding_client.h
        src/path_finding_client.cpp
        )

target_link_libraries(ClientPF2
        ${JSONCPP_LIBRARIES}
        )

add_executable(ClientPO
        demo/demo_path_optimizing_client.cpp
        include/base_client.h
        src/base_client.cpp
        include/path_optimizing_client.h
        src/path_optimizing_client.cpp
        )

target_link_libraries(ClientPO
        ${JSONCPP_LIBRARIES}
        )


add_executable(ClientFTPosition
        demo/demo_tf_position_client.cpp
        include/base_client.h
        src/base_client.cpp
        include/trajectory_finding_client.h
        src/trajectory_finding_client.cpp
        )

target_link_libraries(ClientFTPosition
        ${JSONCPP_LIBRARIES}
        )


add_executable(ClientFTSpeed
        demo/demo_tf_speed_client.cpp
        include/base_client.h
        src/base_client.cpp
        include/trajectory_finding_client.h
        src/trajectory_finding_client.cpp
        )

target_link_libraries(ClientFTSpeed
        ${JSONCPP_LIBRARIES}
        )


add_executable(ClientFTAcceleration
        demo/demo_tf_acceleration_client.cpp
        include/base_client.h
        src/base_client.cpp
        include/trajectory_finding_client.h
        src/trajectory_finding_client.cpp
        )

target_link_libraries(ClientFTAcceleration
        ${JSONCPP_LIBRARIES}
        )
