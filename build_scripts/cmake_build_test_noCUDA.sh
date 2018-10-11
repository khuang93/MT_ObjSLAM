#!/bin/bash
clear

echo "cmake .."
cmake ..  -DWITH_FFMPEG=false -DWITH_CUDA=false -DWITH_OPENMP=true -DWITH_PNG=true

echo "make"
make

echo "Run"
./Mapper ../Dataset/RealisticRenderingDataset
