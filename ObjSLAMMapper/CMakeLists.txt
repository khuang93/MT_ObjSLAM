cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3)

project(ObjSLAMMapper)

set(SOURCE_DIR "src" CACHE STRING "some info" FORCE)




include_directories(${CMAKE_SOURCE_DIR}
        /usr/local/cuda/include
        /usr/include
        ../External/InfiniTAM/InfiniTAM/InputSource
        ../External/InfiniTAM/InfiniTAM/ITMLib
        )

add_subdirectory(../External/InfiniTAM/InfiniTAM build)

set(SOURCES
        src/main.cpp src/DatasetReader_LPD_Dataset.h)

add_executable(ObjSLAMMapper src/main.cpp src/DatasetReader_LPD_Dataset.h)

TARGET_LINK_LIBRARIES(ObjSLAMMapper InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib)