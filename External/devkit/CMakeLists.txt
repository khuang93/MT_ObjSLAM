cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)

project(devkit)


set(devkit_Sources
        /local/MT/MT_ObjSLAM/External/devkit/cpp/evaluate_odometry.cpp
         /local/MT/MT_ObjSLAM/External/devkit/cpp/mail.h
         /local/MT/MT_ObjSLAM/External/devkit/cpp/matrix.cpp
         /local/MT/MT_ObjSLAM/External/devkit/cpp/matrix.h
        )

add_executable(devkit ${devkit_Sources}  /usr/include)