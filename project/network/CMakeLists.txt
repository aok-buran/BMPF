cmake_minimum_required(VERSION  3.5.1)

set(CMAKE_CXX_STANDARD 14)



include_directories(servers/include)
add_subdirectory(servers)

include_directories(clients/include)
add_subdirectory(clients)
