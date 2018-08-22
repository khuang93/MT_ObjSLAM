//
// Created by khuang on 8/22/18.
//

#ifndef MT_OBJSLAM_TEDDYREADER_H
#define MT_OBJSLAM_TEDDYREADER_H

#include "DatasetReader.h"
#include "/local/MT/MT_ObjSLAM/External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMCalibIO.h"

class TeddyReader : public DatasetReader{
 private:
  string calib_path;

 public:
  TeddyReader(string _path, Vector2i _imgSize):DatasetReader(_path, _imgSize){
    cout<<"CreatedTeddyReader"<<endl;
    calib_path = path+"/calib.txt";
    readCalib();
  }

  int readNext();

  ObjSLAM::ObjShortImage *ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth);

  void readCalib();

};

#endif //MT_OBJSLAM_TEDDYREADER_H
