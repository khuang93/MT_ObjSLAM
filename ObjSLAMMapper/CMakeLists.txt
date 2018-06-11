cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3)

project(ObjSLAMMapper)

set(SOURCE_DIR "src" CACHE STRING "some info" FORCE)

include_directories(${CMAKE_SOURCE_DIR})

set(SOURCES
        src/main.cpp src/DatasetReader.h)

add_executable(ObjSLAMMapper src/main.cpp src/DatasetReader.h)

