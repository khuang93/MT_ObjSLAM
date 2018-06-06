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
#add_subdirectory(External/g2o/g2o)

set(ObjSLAM_Sources
#        ${SOURCE_DIR}/ObjSLAMApp.cpp
#        ${SOURCE_DIR}/ObjSLA.MEngine.cpp
#        ${SOURCE_DIR}/InfiniTAMEngine.cpp
#        ${SOURCE_DIR}/ObjectSLAMTracker.cpp
##        ${SOURCE_DIR}/ObjectView.cpp
#        ${SOURCE_DIR}/SemanticLabelEngine.cpp

        src/ObjSLAM/ObjSLAMApp.cpp
        src/ObjSLAM/ObjSLAMEngine.cpp src/ObjSLAM/ObjSLAMEngine.h
        src/ObjSLAM/InfiniTAMEngine.cpp src/ObjSLAM/InfiniTAMEngine.h
        src/ObjSLAM/ObjectSLAMTracker.cpp src/ObjSLAM/ObjectSLAMTracker.h
        src/ObjSLAM/ObjectView.cpp src/ObjSLAM/ObjectView.h
        src/ObjSLAM/ObjCameraPose.cpp src/ObjSLAM/ObjCameraPose.h
        src/ObjSLAM/ObjectClassLabel.cpp src/ObjSLAM/ObjectClassLabel.h
        src/ObjSLAM/ObjectInstance.cpp src/ObjSLAM/ObjectInstance.h)

add_executable(ObjSLAM ${ObjSLAM_Sources})
