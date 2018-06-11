#!/bin/bash
clear

echo "cmake .."
cmake ..

echo "make"
make

echo "run"
./Mapper ../Dataset/RealisticRenderingDataset 1
