//
// Created by khuang on 8/29/18.
//

#include "TUM_Reader.h"

RGB_D_NamePair TUM_Reader::Get_RGB_D_filenames(std::istream &associate_src){
  string depth_name;
  string rgb_name;
  string rgb_num;
  string garbage;

  if(associate_src.eof()) return RGB_D_NamePair{"",""};

//  associate_src>>depth_name;
//  associate_src>>garbage;
//  associate_src>>rgb_name;
//  associate_src>>garbage;

  associate_src>>garbage;
  associate_src>>depth_name;
  associate_src>>rgb_num;
  associate_src>>rgb_name;


  //skip 2 frames
  for(int i = 0; i < reader_SkipFrames; ++i){
    associate_src>>garbage;
    associate_src>>garbage;
    associate_src>>garbage;
    associate_src>>garbage;
  }

  RGB_D_NamePair res;
  res.depth_name=depth_name;
  res.rgb_name=rgb_name;
  res.rgb_num=rgb_num;
  return  res;
}


int TUM_Reader::ReadNext(){
  if (label_img_vector.size() != 0) {
    label_img_vector.clear();
  }

  cout<< "img_number = "<< img_number<<endl;



  RGB_D_NamePair rgbd_name_pair = Get_RGB_D_filenames(associate_f_stream);
  if(rgbd_name_pair.depth_name.empty()&&rgbd_name_pair.rgb_name.empty()){
    cout<<endl<<"No more images, Programm will exit! \n";
    return -1;
  }
//  std::cout<<rgbd_name_pair.rgb_name<<endl;
//  std::cout<<rgbd_name_pair.depth_name<<endl;

  string depth_path = path + "depth/";
  string rgb_path = path + "rgb/";

  string depth_img_path = path + rgbd_name_pair.depth_name;
  string rgb_img_path = path +  rgbd_name_pair.rgb_name;

//  string depth_img_path = depth_path + rgbd_name_pair.depth_name+ ".png";
//  string rgb_img_path = rgb_path +  rgbd_name_pair.rgb_name+ ".png";
  string label_path = path + "/pixel_label/";



  shared_ptr<ObjSLAM::ObjShortImage> disparity_raw =  ReadOneDisparity(depth_img_path);

  std::cout<<rgb_img_path<<endl;
  std::cout<<depth_img_path<<endl;

  depth_img = std::make_shared<ObjSLAM::ObjFloatImage>(imgSize,MEMORYDEVICE_CPU);
  viewBuilder->ConvertDepthAffineToFloat(depth_img.get(),disparity_raw.get(),calib->disparityCalib.GetParams());
//  delete disparity_raw;
//  std::cout<<"depth"<<depth_img->GetData(MEMORYDEVICE_CPU)[0]<<endl;
//  SaveImageToFile(depth_img, "testDepth.ppm");

  rgb_img = ReadOneRGB(rgb_img_path);

  //read labels
//  std::vector<string> label_fileNames = GetFileNames(label_path);
  std::vector<string> filteredNames;
  string prefix = rgbd_name_pair.rgb_name.erase(0,4) /*+ ".png."*/;
  for (int i = 0; i < LabelFileNames.size(); i++) {
//    std::cout<<LabelFileNames.at(i)<<endl;

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

ObjSLAM::ObjShortImage *TUM_Reader::ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth){
  return nullptr;
}