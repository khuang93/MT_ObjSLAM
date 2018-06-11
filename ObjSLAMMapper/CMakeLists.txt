cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)


project(ObjSLAMMapper)

set(SOURCE_DIR "src" CACHE STRING "some info" FORCE)

include_directories(${SOURCE_DIR})

set(SOURCES
        src/main.cpp)

add_executable(ObjSLAMMapper ${SOURCES})

