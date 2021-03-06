//
// Created by khuang on 8/29/18.
//

#ifndef MT_OBJSLAM_TUM_READER_H
#define MT_OBJSLAM_TUM_READER_H

#include "DatasetReader.h"

#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/ViewBuilding/CPU/ITMViewBuilder_CPU.h>
#include "DatasetReader.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMCalibIO.h"

struct RGB_D_NamePair{
  string rgb_name;
  string rgb_num;
  string depth_name;
};

class TUM_Reader : public DatasetReader{
protected:
  string calib_path;
  std::ifstream associate_f_stream;
 public:
  TUM_Reader(string _path, Vector2i _imgSize):DatasetReader(_path, _imgSize){

    img_number=1;
    calib_path = path+"calib.txt";
      ReadCalib(calib_path);
    string associate_file_name = path + "/associate.txt";
    associate_f_stream.open(associate_file_name);
    viewBuilder = new ITMLib::ITMViewBuilder_CPU(*calib);
    cout<<"Created TUM_Reader Path "<<path<<endl;

  }

  ~TUM_Reader(){

  }

  int ReadNext();

  RGB_D_NamePair Get_RGB_D_filenames(std::istream &associate_src);




  ObjSLAM::ObjShortImage *ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth);

//  bool ReadCalib(string calib_path);


};

#endif //MT_OBJSLAM_TUM_READER_H
