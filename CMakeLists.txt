cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)

project(MT_ObjSLAM)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set (PROJECT_DIR "/local/MT/MT_ObjSLAM")
set (SOURCE_DIR "src/ObjSLAM" CACHE STRING "some info" FORCE)
set (HEADER_DIR "src/ObjSLAM" CACHE STRING "some info" FORCE)


#set (INFINITAM_DIR "External/InfiniTAM/InfiniTAM")


#from Apps/InfiniTAM/CMakeLists.txt
#INCLUDE(${INFINITAM_DIR}/cmake/UseCUDA.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseFFmpeg.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseGLUT.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseLibRoyale.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseOpenGL.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseOpenMP.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseOpenNI.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UsePNG.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseRealSense.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseRealSense2.cmake)
#INCLUDE(${INFINITAM_DIR}/cmake/UseUVC.cmake)
find_package( Boost )

include_directories($(HEADER_DIR)
        /usr/local/cuda/include
        /usr/include
        External/InfiniTAM/InfiniTAM/InputSource
        External/InfiniTAM/InfiniTAM/ITMLib
        )



add_subdirectory(External/InfiniTAM/InfiniTAM)

#add_subdirectory(External/g2o/g2o)



set(ObjSLAM_Sources
        src/ObjSLAM/ObjSLAMMapper_Test.cpp
        src/ObjSLAM/ObjSLAMEngine.h
        src/ObjSLAM/ObjSLAMEngine.tpp
        src/ObjSLAM/InfiniTAMEngine.cpp src/ObjSLAM/InfiniTAMEngine.h
        src/ObjSLAM/ObjectSLAMTracker.cpp src/ObjSLAM/ObjectSLAMTracker.h
        src/ObjSLAM/ObjectView_old.cpp src/ObjSLAM/ObjectView_old.h
        src/ObjSLAM/ObjectClassLabel.cpp src/ObjSLAM/ObjectClassLabel.h
        src/ObjSLAM/ObjectInstance.cpp src/ObjSLAM/ObjectInstance.h
        src/ObjSLAM/ObjectInstanceScene.cpp src/ObjSLAM/ObjectInstanceScene.h
        src/ObjSLAM/ObjSLAMDataTypes.h

        src/ObjSLAM/DatasetReader_LPD_Dataset.h
        src/ObjSLAM/ObjCameraPose.cpp src/ObjSLAM/ObjCameraPose.h
        src/ObjSLAM/LPD_RAW_Pose.cpp src/ObjSLAM/LPD_RAW_Pose.h
        src/ObjSLAM/ObjSceneReconstructionEngine_CPU.cpp src/ObjSLAM/ObjSceneReconstructionEngine_CPU.h
        src/ObjSLAM/ObjectView_New.cpp src/ObjSLAM/ObjectView_New.h
        #[[        src/ObjSLAM/ObjSLAMBasicEngine.cpp src/ObjSLAM/ObjSLAMBasicEngine.h]]
        src/ObjSLAM/ObjSLAMMappingEngine.tpp src/ObjSLAM/ObjSLAMMappingEngine.h
        )
set(Mapper_Sources
        src/ObjSLAM/ObjSLAMMapper_Test.cpp

        src/ObjSLAM/InfiniTAMEngine.cpp src/ObjSLAM/InfiniTAMEngine.h
        src/ObjSLAM/ObjectSLAMTracker.cpp src/ObjSLAM/ObjectSLAMTracker.h
        src/ObjSLAM/ObjectView_old.cpp src/ObjSLAM/ObjectView_old.h
        src/ObjSLAM/ObjectClassLabel.cpp src/ObjSLAM/ObjectClassLabel.h
        src/ObjSLAM/ObjectInstance.cpp src/ObjSLAM/ObjectInstance.h
        src/ObjSLAM/ObjectInstanceScene.cpp src/ObjSLAM/ObjectInstanceScene.h
        src/ObjSLAM/ObjSLAMDataTypes.h

        src/ObjSLAM/DatasetReader_LPD_Dataset.h
        src/ObjSLAM/ObjCameraPose.cpp src/ObjSLAM/ObjCameraPose.h
        src/ObjSLAM/LPD_RAW_Pose.cpp src/ObjSLAM/LPD_RAW_Pose.h
        src/ObjSLAM/ObjSceneReconstructionEngine_CPU.cpp src/ObjSLAM/ObjSceneReconstructionEngine_CPU.h
        src/ObjSLAM/ObjectView_New.cpp src/ObjSLAM/ObjectView_New.h
#[[        src/ObjSLAM/ObjSLAMBasicEngine.cpp src/ObjSLAM/ObjSLAMBasicEngine.h]]
        src/ObjSLAM/ObjSLAMMappingEngine.tpp src/ObjSLAM/ObjSLAMMappingEngine.h src/ObjSLAM/IOUtils.cpp src/ObjSLAM/IOUtils.h)

#add_executable(ObjSLAM ${ObjSLAM_Sources})
#TARGET_LINK_LIBRARIES(ObjSLAM cudart InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib Threads::Threads png)



add_executable(Mapper ${Mapper_Sources}  /usr/include)
TARGET_LINK_LIBRARIES(Mapper  boost_system  boost_filesystem  cudart InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib Threads::Threads png)
#looks like it needs link cudart
