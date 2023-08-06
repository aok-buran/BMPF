cmake_minimum_required(VERSION  3.5.1)
project(misc)

add_compile_options(-std=c++14)


include_directories(
        include
)

#
#add_library(
#        ${PROJECT_NAME}
#        src/traces.cpp
#)

add_library(
        ${PROJECT_NAME}
        src/state.cpp
        include/count_down_latch.h
        include/safe_ptr.h
        include/matrix_math.h
        include/state.h
)