//
// Created by khuang on 8/29/18.
//

#include "TUM_Reader.h"

RGB_D_NamePair TUM_Reader::get_RGB_D_filenames(std::istream & associate_src){
  string depth_name;
  string rgb_name;
  string garbage;

  associate_src>>depth_name;
  associate_src>>garbage;
  associate_src>>rgb_name;
  associate_src>>garbage;

  RGB_D_NamePair res;
  res.depth_name=depth_name;
  res.rgb_name=rgb_name;
  return  res;
}


int TUM_Reader::readNext(){
  cout<< "img_number = "<< img_number<<endl;



  RGB_D_NamePair rgbd_name_pair = get_RGB_D_filenames(associate_f_stream);

//  std::cout<<rgbd_name_pair.rgb_name<<endl;
//  std::cout<<rgbd_name_pair.depth_name<<endl;

  string depth_path = path + "depth/";
  string rgb_path = path + "rgb/";

  string depth_img_path = depth_path + rgbd_name_pair.depth_name+ ".png";
  string rgb_img_path = rgb_path +  rgbd_name_pair.rgb_name+ ".png";
  string label_path = path + "/pixel_label/";



  ObjSLAM::ObjShortImage* disparity_raw =  ReadOneDisparity(depth_img_path);

//  std::cout<<rgb_img_path<<endl;
//  std::cout<<depth_img_path<<endl;

  depth_img = new ObjSLAM::ObjFloatImage(imgSize,MEMORYDEVICE_CPU);
  viewBuilder->ConvertDepthAffineToFloat(depth_img,disparity_raw,calib->disparityCalib.GetParams());
  delete disparity_raw;
//  std::cout<<"depth"<<depth_img->GetData(MEMORYDEVICE_CPU)[0]<<endl;
//  SaveImageToFile(depth_img, "testDepth.ppm");

  rgb_img = ReadOneRGB(rgb_img_path);

  //read labels
  std::vector<string> label_fileNames = getFileNames(label_path);
  std::vector<string> filteredNames;

  for (int i = 0; i < label_fileNames.size(); i++) {
//    std::cout<<label_fileNames.at(i)<<endl;
    string prefix = rgbd_name_pair.rgb_name + ".png.";
    if (boost::starts_with(label_fileNames.at(i), prefix) && label_fileNames.at(i) != prefix) {
      filteredNames.push_back(label_fileNames.at(i));
    }
  }

  std::sort(filteredNames.begin(), filteredNames.end());

  for (int i = 0; i < filteredNames.size(); i++) {

    label_img_vector.push_back(ReadLabel_OneFile(label_path + filteredNames.at(i)));
  }




  return img_number++;
}

ObjSLAM::ObjShortImage *TUM_Reader::ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth){
  return nullptr;
}