//
// Created by khuang on 8/22/18.
//

#include "TeddyReader.h"
#include "/local/MT/MT_ObjSLAM/External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMCalibIO.h"

int TeddyReader::readNext(){
  return 0;
}

ObjSLAM::ObjShortImage *TeddyReader::ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth){
  return nullptr;
}

void TeddyReader::readCalib(){
  ITMLib::readRGBDCalib(calib_path.c_str(), *calib);
}