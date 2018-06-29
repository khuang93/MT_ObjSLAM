//
// Created by khuang on 6/11/18.
//

#ifndef DATASETREADER_LPD_DATASET_H
#define DATASETREADER_LPD_DATASET_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>

//#include <opencv2/opencv.hpp>
#include "ObjSLAMDataTypes.h"
#include "ObjCameraPose.h"

#include "../../External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

//#include <eigen3/Eigen/Dense>

using namespace std;

class DatasetReader_LPD_Dataset {
 private:
  int width, height;
  ITMLib::ITMRGBDCalib *calib;

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
  Vector2i getSize() {
    Vector2i res(width, height);
    return res;
  }

  ObjSLAM::ObjFloatImage *ReadDepth(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<float> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
    cout << "**Input size is " << vector_in.size() << endl;

    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjFloatImage(newDims, MEMORYDEVICE_CPU);

    res->ChangeDims(newDims);


//
    for (int i = 0; i < height; i++) {
//      cout<<"i"<<i<<endl;
      for (int j = 0; j < width; j++) {
//        res[height * i + j] = vector_in.at(height * i + j);

        res->GetData(MEMORYDEVICE_CPU)[width * i + j] = vector_in.at(height * i + j);
      }

    }

//    SaveImageToFile(res, "testD");
    return res;

  }

  ObjSLAM::ObjFloatImage *convertRayDepthToZDepth(ObjSLAM::ObjFloatImage *in) {
    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjFloatImage(newDims, MEMORYDEVICE_CPU);

    Eigen::Matrix3f K;/* = Eigen::Matrix3d::Zero();*/
    K(0,0)= this->calib->intrinsics_d.projectionParamsSimple.fx;

    K(1, 1) = this->calib->intrinsics_d.projectionParamsSimple.fy;
    K(0, 2) = this->calib->intrinsics_d.projectionParamsSimple.px;
    K(1, 2) = this->calib->intrinsics_d.projectionParamsSimple.py;
    K(2, 2) = 1;

    Eigen::Matrix3f K_inv = K.inverse();

    //TODO: Make this a camera Projection function
    for(int i = 0; i < height;i++){
      for(int j = 0; j< width;j++){
        Eigen::Vector3f Pix(j,i,1.0);
        Eigen::Vector3f WorldVec = K_inv*Pix;
        WorldVec.normalize();
        WorldVec = WorldVec*in->GetElement(i*width+j,MEMORYDEVICE_CPU);
        res->GetData(MEMORYDEVICE_CPU)[i*width+j]=WorldVec(2);
      }
    }
    return  res;
  }

  ObjSLAM::ObjUChar4Image *ReadRGB(std::string Path) {

    ifstream in;

    in.open(Path);
    //read rgb from png file


    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjUChar4Image(newDims, MEMORYDEVICE_CPU);

    res->ChangeDims(newDims);

    ReadImageFromFile(res, Path.c_str());

//    SaveImageToFile(res, "testRGB");


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

  ObjSLAM::ObjUIntImage *ReadLabel_OneFile(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<unsigned int> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
    cout << "**Input size of label is " << vector_in.size() << endl;

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

  ObjSLAM::LPD_RAW_Pose *ReadLPDRawPose(std::string Path, double t) {
    ifstream in;
    in.open(Path);

    ObjSLAM::LPD_RAW_Pose *res = new ObjSLAM::LPD_RAW_Pose();

    string currentLine;
    //TODO get a more efficient way to do the read in instead of loop from begin every time...(one loop to read in every time step)
    while (getline(in, currentLine)) {
      istringstream iss(currentLine);
      double currentT = 0.0;
      iss >> currentT;
      if (currentT == t) {
        iss >> res->qw;
        iss >> res->qx;
        iss >> res->qy;
        iss >> res->qz;
        iss >> res->x;
        iss >> res->y;
        iss >> res->z;
        iss >> res->vx;
        iss >> res->vy;
        iss >> res->vz;
        iss >> res->p;
        iss >> res->q;
        iss >> res->r;
        iss >> res->ax;
        iss >> res->ay;
        iss >> res->az;

        break;
      }
    }
    return res;
  }

  ObjSLAM::ObjCameraPose *convertRawPose_to_Pose(ObjSLAM::LPD_RAW_Pose *_rawPose) {
    auto *res = new ObjSLAM::ObjCameraPose(
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

  ObjSLAM::ObjShortImage *calculateDisparityFromDepth(ObjSLAM::ObjFloatImage *depth) {

    ORUtils::Vector2<int> newDims(width, height);
    auto *disparity = new ObjSLAM::ObjShortImage(newDims, MEMORYDEVICE_CPU);

    float fx = this->calib->intrinsics_d.projectionParamsSimple.fx;
//    cout<<"fx"<<fx;
    float bxf = 8.0f * this->calib->disparityCalib.GetParams().y * fx;

    for (int y = 0; y < newDims.y; y++) {
      for (int x = 0; x < newDims.x; x++) {
        int locId = x + y * newDims.x;

        float depth_pixel = depth->GetData(MEMORYDEVICE_CPU)[locId];
        float disparity_tmp = bxf / depth_pixel;
        int disparity_pixel = this->calib->disparityCalib.GetParams().x - disparity_tmp;

//        cout<<disparity_pixel;
      }
    }
  }

  void setCalib_LPD() {
    calib = new ITMLib::ITMRGBDCalib();

    calib->intrinsics_rgb.SetFrom(640, 480, 320, 320, 320, 240);
    calib->intrinsics_d.SetFrom(640, 480, 320, 320, 320, 240);

    ObjSLAM::ObjMatrix4f calib_Ext;
    calib_Ext.m00 = 0.5;
    calib_Ext.m10 = 0;
    calib_Ext.m20 = 0;
    calib_Ext.m30 = 0;
    calib_Ext.m01 = 0;
    calib_Ext.m11 = 0.5;
    calib_Ext.m21 = 0;
    calib_Ext.m31 = 0;
    calib_Ext.m02 = 0;
    calib_Ext.m12 = 0;
    calib_Ext.m22 = 0.5;
    calib_Ext.m32 = 0;
    calib_Ext.m03 = 0;
    calib_Ext.m13 = 0;
    calib_Ext.m23 = 0;
    calib_Ext.m33 = 1;
    calib->trafo_rgb_to_depth.SetFrom(calib_Ext);

    //Calib Extrinsics is between RGB and D, for LPD Dataset it is identity
    ObjSLAM::ObjMatrix4f mat(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    calib->trafo_rgb_to_depth.SetFrom(mat);

    //disparity calib a b physical meanings?
    calib->disparityCalib.SetFrom(0.01, 0.4, ITMLib::ITMDisparityCalib::TRAFO_AFFINE); //TODO get the values


  }

//  void readExtrnsics(string Path){
//
//    ObjSLAM::ObjMatrix4f calib_Ext;
//    Eigen::Quaterniond R(0.5, -0.5,0.5,-0.5);
//    Eigen::Matrix3d eigen_mat = R.normalized().toRotationMatrix();
//
//    double m00 = eigen_mat(0,0);double m01 = eigen_mat(0,1);double m02 = eigen_mat(0,2);
//    double m10 = eigen_mat(1,0);double m11 = eigen_mat(1,1);double m12 = eigen_mat(1,2);
//    double m20 = eigen_mat(2,0);double m21 = eigen_mat(2,1);double m22 = eigen_mat(2,2);
//    ObjSLAM::ObjMatrix4f mat(1,0,0, 0,0,1,0, 0,0,0,1,0,0,0,0,1);
//    calib->trafo_rgb_to_depth.SetFrom(mat);
//  }



  ITMLib::ITMRGBDCalib *getCalib() {
    return calib;
  }

};

#endif //DATASETREADER__LPD_DATASET_H
