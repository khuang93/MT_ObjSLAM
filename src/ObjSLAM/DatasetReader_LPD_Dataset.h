//
// Created by khuang on 6/11/18.
//

#ifndef DATASETREADER_LPD_DATASET_H
#define DATASETREADER_LPD_DATASET_H

//#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>

//#include <opencv2/opencv.hpp>
#include "ObjSLAMDataTypes.h"
#include "ObjCameraPose.h"



#include "../../External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
//#include "eigen3/Eigen/Geometry"

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

    SaveImageToFile(res, "testD");
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

    SaveImageToFile(res, "testRGB");


    return res;
  }

/*  ObjSLAM::ObjFloat4Image* ReadNormal(std::string Path) {

    ifstream in;

    in.open(Path);
    //read rgb from png file


    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjFloat4Image(newDims, MEMORYDEVICE_CPU);


    res->ChangeDims(newDims);

    ReadImageFromFile(res, Path.c_str());


    return res;
  }*/

  ObjSLAM::ObjUIntImage* ReadLabel(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<unsigned int> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
    cout << "**Input size of label is "<<vector_in.size()<<endl;

    //set dimension
    ORUtils::Vector2<int> newDims(width, height);

    //create UintImage object
    auto *res = new ObjSLAM::ObjUIntImage(newDims, MEMORYDEVICE_CPU);

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

  ObjSLAM::LPD_RAW_Pose* ReadLPDRawPose(std::string Path, double t){
    ifstream in;
    in.open(Path);

    ObjSLAM::LPD_RAW_Pose* res = new ObjSLAM::LPD_RAW_Pose();

    string currentLine;
    //TODO get a more efficient way to do the read in instead of loop from begin every time...(one loop to read in every time step)
    while(getline(in, currentLine)){
      istringstream iss(currentLine);
      double currentT=0.0;
      iss>>currentT;
      if(currentT==t) {
        iss>>res->qw;
        iss>>res->qx;
        iss>>res->qy;
        iss>>res->qz;
        iss>>res->x;
        iss>>res->y;
        iss>>res->z;
        iss>>res->vx;
        iss>>res->vy;
        iss>>res->vz;
        iss>>res->p;
        iss>>res->q;
        iss>>res->r;
        iss>>res->ax;
        iss>>res->ay;
        iss>>res->az;

        break;
      }
    }
    return res;
  }

  ObjSLAM::ObjCameraPose* convertRawPose_to_Pose(ObjSLAM::LPD_RAW_Pose* _rawPose){
    auto* res = new ObjSLAM::ObjCameraPose(
        _rawPose->qw,
        _rawPose->qx,
        _rawPose->qy,
        _rawPose->qz,
        _rawPose->x,
        _rawPose->y,
        _rawPose->z
        );

    return res;

  }

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
