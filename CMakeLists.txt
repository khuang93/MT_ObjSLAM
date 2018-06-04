cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

project(MT_ObjSLAM)

set (PROJECT_DIR "/local/MT/MT_ObjSLAM")
set (SOURCE_DIR "src/ObjSLAM" CACHE STRING "some info" FORCE)
set (HEADER_DIR "src/ObjSLAM" CACHE STRING "some info" FORCE)


#add_subdirectory()

include_directories(AFTER $(SOURCE_DIR))

add_executable(ObjSLAM ${SOURCE_DIR}/ObjSLAMApp.cpp)
