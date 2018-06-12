//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>
#include <string>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjSLAMDataTypes.h"

using namespace std;

int main(int argc, char** argv){
  //TODO Debug output
  cout<<"**Hello SLAM World!"<<endl;

  //Path of the depth image file
  string path = argv[1];


  std::istringstream iss( argv[2] );
  //for LPD dataset 1 -13
  int img_number;
  iss>>img_number;

  //TODO make the path using os path join instead of slash
  string depth_path =path + "/depth/cam0/" + to_string(img_number) + ".exr";
  string rgb_path =path + "/rgb/cam0/" + to_string(img_number) + ".png";

  cout<<"depth path "<<depth_path<<endl;

  DatasetReader_LPD_Dataset reader(640,480);
//  cout<<reader.getHeight()<<" "<<reader.getWidth()<<endl;


  ObjSLAM::ObjFloatImage* depth_img = reader.ReadDepth(depth_path);
  ObjSLAM::ObjUChar4Image* rgb_img = reader.ReadRGB(rgb_path);

  //TODO Debug output
  cout <<"** Debug: "<<depth_img->GetElement(0, MEMORYDEVICE_CPU)<<endl;
  cout <<"** Debug: "<<rgb_img->GetElement(35000, MEMORYDEVICE_CPU).r<<endl;
  return 0;
}
