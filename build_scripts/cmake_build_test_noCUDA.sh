#!/bin/bash
clear

echo "cmake .."
cmake ..  -DWITH_FFMPEG=false -DWITH_CUDA=false

echo "make"
make

echo "run"
./Mapper ../Dataset/RealisticRenderingDataset
