//
// Created by khuang on 8/22/18.
//

#include "TeddyReader.h"
//#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMCalibIO.h"
#include <iomanip>

int TeddyReader::readNext(){
  if (label_img_vector.size() != 0) {
    label_img_vector.clear();
  }


  cout<< "img_number = "<< img_number<<endl;
  std::stringstream img_num_mod;
  img_num_mod<<setfill('0')<<setw(4)<<img_number;
  string img_num_mod_str = img_num_mod.str();
  string depth_path = path + "/"+ img_num_mod_str+ ".pgm";
  string rgb_path = path + "/"+ img_num_mod_str+ ".ppm";
  string label_path = path + "/pixel_label/";



  ObjSLAM::ObjShortImage* disparity_raw =  ReadOneDisparity(depth_path);

//  std::cout<<depth_path<<"value"<<disparity_raw->GetData(MEMORYDEVICE_CPU)[0]<<endl;

  depth_img = new ObjSLAM::ObjFloatImage(imgSize,MEMORYDEVICE_CPU);
  viewBuilder->ConvertDisparityToDepth(depth_img,disparity_raw,&(calib->intrinsics_d),calib->disparityCalib.GetParams());
  delete disparity_raw;


//  std::cout<<"depth"<<depth_img->GetData(MEMORYDEVICE_CPU)[0]<<endl;
  SaveImageToFile(depth_img, "testDepth.ppm");

  rgb_img = ReadOneRGB(rgb_path);


  //read labels
//  std::vector<string> fileNames = getFileNames(label_path);
  std::vector<string> filteredNames;

  for (int i = 0; i < LabelFileNames.size(); i++) {
    string prefix = img_num_mod_str + ".";
    if (boost::starts_with(LabelFileNames.at(i), prefix) && LabelFileNames.at(i) != prefix) {
      filteredNames.push_back(LabelFileNames.at(i));
    }
  }

  std::sort(filteredNames.begin(), filteredNames.end());

  for (int i = 0; i < filteredNames.size(); i++) {
    label_img_vector.push_back(ReadLabel_OneFile(label_path + filteredNames.at(i)));
  }




  return img_number++;
}

ObjSLAM::ObjShortImage *TeddyReader::ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth){
  return nullptr;
}

/*
bool TeddyReader::readCalib(){


  readCalib(this->calib_path);

  return true;
}
*/
