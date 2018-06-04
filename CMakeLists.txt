cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

project(MT_ObjSLAM)

set (PROJECT_DIR "/local/MT/MT_ObjSLAM")
set (SOURCE_DIR "src/ObjSLAM" CACHE STRING "some info" FORCE)
set (HEADER_DIR "src/ObjSLAM" CACHE STRING "some info" FORCE)


#add_subdirectory()

include_directories($(HEADER_DIR)
        /usr/local/cuda/include
        /usr/include
        )

add_subdirectory(External/InfiniTAM/InfiniTAM)
add_subdirectory(External/g2o/g2o)

set(ObjSLAM_Sources
        ${SOURCE_DIR}/ObjSLAMApp.cpp
        ${SOURCE_DIR}/ObjSLAMEngine.cpp
        ${SOURCE_DIR}/InfiniTAMEngine.cpp
        ${SOURCE_DIR}/ObjectSLAMTracker.cpp
#        ${SOURCE_DIR}/ObjectView.cpp

    )

add_executable(ObjSLAM ${ObjSLAM_Sources})
