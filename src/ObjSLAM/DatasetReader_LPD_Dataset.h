//
// Created by khuang on 6/11/18.
//

#ifndef DATASETREADER_LPD_DATASET_H
#define DATASETREADER_LPD_DATASET_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
//#include <opencv2/opencv.hpp>
#include "ObjSLAMDataTypes.h"
#include <stdlib.h>


#include "../../External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"

using namespace std;

class DatasetReader_LPD_Dataset {
 private:
  int width, height;
  ITMLib::ITMRGBDCalib* calib;


 public:
  DatasetReader_LPD_Dataset() {};

  DatasetReader_LPD_Dataset(int w, int h) : width(w), height(h) {};

  void setWidth(int w) {
    width = w;
  }
  void setHeight(int h) {
    height = h;
  }
  int getWidth() {
    return width;
  }
  int getHeight() {
    return height;
  }
  Vector2i getSize(){
    Vector2i res(width,height);
    return res;
  }

  ObjSLAM::ObjFloatImage* ReadDepth(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<float> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
    cout << "**Input size is "<<vector_in.size()<<endl;

    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjFloatImage(newDims, MEMORYDEVICE_CPU);

    res->ChangeDims(newDims);


//
    for (int i = 0; i < width; i++) {
//      cout<<"i"<<i<<endl;
      for (int j = 0; j < height; j++) {
//        res[height * i + j] = vector_in.at(height * i + j);

        res->GetData(MEMORYDEVICE_CPU)[height * i + j] = vector_in.at(height * i + j);
      }

    }
    return res;

  }

  ObjSLAM::ObjUChar4Image* ReadRGB(std::string Path) {

    ifstream in;

    in.open(Path);
  //read rgb from png file


    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjUChar4Image(newDims, MEMORYDEVICE_CPU);


    res->ChangeDims(newDims);

    ReadImageFromFile(res, Path.c_str());


    return res;
  }

  ObjSLAM::ObjUIntImage* ReadLabel(std::St)


  void setCalib_LPD(){
    calib = new ITMLib::ITMRGBDCalib();


    calib->intrinsics_rgb.SetFrom(640,480,320,320,320,240);
    calib->intrinsics_d.SetFrom(640,480,320,320,320,240);

    ObjSLAM::ObjMatrix4f calib_Ext;
    calib_Ext.m00=0.5;calib_Ext.m10=0;calib_Ext.m20=0;calib_Ext.m30=0;
    calib_Ext.m01=0;calib_Ext.m11=0.5;calib_Ext.m21=0;calib_Ext.m31=0;
    calib_Ext.m02=0;calib_Ext.m12=0;calib_Ext.m22=0.5;calib_Ext.m32=0;
    calib_Ext.m03=0;calib_Ext.m13=0;calib_Ext.m23=0;calib_Ext.m33=1;
    calib->trafo_rgb_to_depth.SetFrom(calib_Ext);
    calib->disparityCalib.SetFrom(0.0,0.0,ITMLib::ITMDisparityCalib::TRAFO_AFFINE);
  }
  
  ITMLib::ITMRGBDCalib* getCalib(){
    return calib;
  }

};

#endif //DATASETREADER__LPD_DATASET_H
