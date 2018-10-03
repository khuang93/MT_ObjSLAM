#!/bin/bash
clear

echo "Run ITM with LPD Data ..."
/local/MT/MT_ObjSLAM/External/InfiniTAM/InfiniTAM/build/Apps/InfiniTAM/InfiniTAM /local/MT/MT_ObjSLAM/Dataset/LPD/calib_2.txt /local/MT/MT_ObjSLAM/Dataset/LPD/%04i.ppm /local/MT/MT_ObjSLAM/Dataset/LPD/%04i.pgm
