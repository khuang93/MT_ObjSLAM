#!/bin/bash
clear

echo "cmake .."
cmake ..  -DWITH_FFMPEG=false -DWITH_CUDA=false -DWITH_OPENMP=false -DWITH_PNG=true

echo "make"
make

echo "run"
./Mapper ../Dataset/RealisticRenderingDataset
