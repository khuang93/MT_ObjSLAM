#!/bin/bash
clear

echo "cmake .."
cmake ..  -DWITH_FFMPEG=false #-DWITH_CUDA=false

echo "make"
make

echo "Run"
./Mapper ../Dataset/RealisticRenderingDataset 1
