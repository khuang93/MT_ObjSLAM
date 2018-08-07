//
// Created by khuang on 6/11/18.
//

#ifndef DATASETREADER_LPD_DATASET_H
#define DATASETREADER_LPD_DATASET_H


#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <memory>
//#include <opencv2/opencv.hpp>
#include "ObjSLAMDataTypes.h"
#include "ObjCameraPose.h"

#include "../../External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

using namespace std;

using LabelImgVector = std::vector<std::shared_ptr<ObjSLAM::ObjUIntImage>>;

class DatasetReader_LPD_Dataset {
 private:
  int width, height;
  Vector2i imgSize;
  ITMLib::ITMRGBDCalib *calib;
  ifstream pose_in;
  ObjSLAM::ObjCameraPose *pose_wc;
  string path;
 public:
  ObjSLAM::ObjUChar4Image *rgb_img;
  ObjSLAM::ObjFloatImage *depth_img;
  std::shared_ptr<ObjSLAM::ObjUIntImage> label_img;
  LabelImgVector label_img_vector;

//  ObjSLAM::ObjUChar4Image *rgb_img_prev;
//  ObjSLAM::ObjFloatImage *depth_img_prev;
//  ObjSLAM::ObjUIntImage *label_img_prev;


//  ObjSLAM::ObjCameraPose* pose_cw_prev;

  int img_number = 1;

 public:
  DatasetReader_LPD_Dataset() {};

  DatasetReader_LPD_Dataset(string _path, int w, int h) : path(_path), width(w), height(h) {
    imgSize = Vector2i(w, h);
    setCalib_LPD();
  };

  DatasetReader_LPD_Dataset(string _path, Vector2i _imgSize) : path(_path), imgSize(_imgSize) {
    width = imgSize.x;
    height = imgSize.y;
    setCalib_LPD();
  };

  int readNext() {
    cout << "img_number = " << img_number << endl;
//    if (img_number > 1) {
//      deleteVariables();
//    }
    if(label_img_vector.size()!=0){
      label_img_vector.clear();
    }

    //TODO make the path using os path join instead of slash
    string depth_path = path + "/depth/cam0/" + to_string(img_number) + ".exr";
    string rgb_path = path + "/rgb/cam0/" + to_string(img_number) + ".png";
    string normal_path = path + "/normal/cam0/" + to_string(img_number) + ".png";
    string label_path = path + "/pixel_label/cam0/" + to_string(img_number) + ".txt";
    string pose_path = path + "/groundTruthPoseVel_imu.txt";

    std::vector<string> fileNames = getFileNames(path + "/pixel_label/cam0/");
    std::vector<string> filteredNames;

    for (int i = 0; i < fileNames.size(); i++) {
      string prefix = to_string(img_number) + ".";
      if (boost::starts_with(fileNames.at(i), prefix) && fileNames.at(i) != prefix) {
        filteredNames.push_back(fileNames.at(i));
      }
    }
    std::sort(filteredNames.begin(), filteredNames.end());

    for (int i = 0; i < filteredNames.size(); i++) {
      label_img_vector.push_back(ReadLabel_OneFile(path + "/pixel_label/cam0/" + filteredNames.at(i)));
    }

    if (!pose_in.is_open()) {
      pose_in.open(pose_path);
//      cout << "ifstream open: " << pose_path << endl;
    }

    //depth
    ObjSLAM::ObjFloatImage *ray_depth_img = ReadOneDepth(depth_path);

    depth_img = convertRayDepthToZDepth(ray_depth_img);
    delete ray_depth_img;
//    string name = to_string(img_number)+".ppm";
//    SaveImageToFile(depth_img,name.c_str());
    rgb_img = ReadOneRGB(rgb_path);

    label_img = ReadLabel_OneFile(label_path);

    double time = img_number * 0.1;

    ObjSLAM::LPD_RAW_Pose *raw_pose = ReadLPDRawPose(pose_in, time);
    //T_bw
    ObjSLAM::ObjCameraPose *T_bw = convertRawPose_to_Pose(raw_pose);
//    cout<<T_bw->getSE3Pose().GetM();
    //T_cb
    ObjSLAM::ObjCameraPose *T_cb = new ObjSLAM::ObjCameraPose(0.5, -0.5, 0.5, -0.5, 0, 0, 0);
//    cout<<T_cb->getSE3Pose().GetM();
    //Cam to World
//    ORUtils::SE3Pose T_cw_SE3 = T_cb->getSE3Pose().GetM() * T_bw->getSE3Pose().GetM();
    ORUtils::SE3Pose T_cw_SE3 = T_bw->getSE3Pose().GetM() * T_cb->getSE3Pose().GetM();

    ORUtils::SE3Pose T_wc_SE3(T_cw_SE3.GetInvM());

    //world to cam
    pose_wc = new ObjSLAM::ObjCameraPose(T_wc_SE3);

    delete raw_pose;
    delete T_bw;
    delete T_cb;
//    delete T_cw_SE3;


    return img_number++;
  }

  std::vector<std::string> getFileNames(std::string directoryPath) {
    namespace fs = boost::filesystem;
    std::vector<std::string> names;

    if (fs::exists(directoryPath)) {
      fs::directory_iterator it(directoryPath);
      fs::directory_iterator end;

      while (it != end) {
        names.push_back(it->path().filename().string());
        ++it;
      }
    }

    return names;
  }

  ObjSLAM::ObjFloatImage *ReadOneDepth(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<float> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
//    cout << "**Input size is " << vector_in.size() << endl;

//    ORUtils::Vector2<int> imgSize(width, height);
    auto *res = new ObjSLAM::ObjFloatImage(imgSize, MEMORYDEVICE_CPU);

    res->ChangeDims(imgSize);


//
    for (int i = 0; i < height; i++) {
//      cout<<"i"<<i<<endl;
      for (int j = 0; j < width; j++) {
//        res[height * i + j] = vector_in.at(height * i + j);

        res->GetData(MEMORYDEVICE_CPU)[width * i + j] = vector_in.at(width * i + j); //in meters
      }

    }

//    SaveImageToFile(res, "testD");
    return res;

  }

  ObjSLAM::ObjFloatImage *convertRayDepthToZDepth(ObjSLAM::ObjFloatImage *in) {
//    ORUtils::Vector2<int> imgSize(width, height);
    auto *res = new ObjSLAM::ObjFloatImage(imgSize, MEMORYDEVICE_CPU);

    Eigen::Matrix3f K;/* = Eigen::Matrix3d::Zero();*/
    K(0, 0) = this->calib->intrinsics_d.projectionParamsSimple.fx;

    K(1, 1) = this->calib->intrinsics_d.projectionParamsSimple.fy;
    K(0, 2) = this->calib->intrinsics_d.projectionParamsSimple.px;
    K(1, 2) = this->calib->intrinsics_d.projectionParamsSimple.py;
    K(2, 2) = 1;

    Eigen::Matrix3f K_inv = K.inverse();

    //TODO: Make this a camera Projection function
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        Eigen::Vector3f Pix(j, i, 1.0);
        Eigen::Vector3f WorldVec = K_inv * Pix;
        WorldVec.normalize();
        WorldVec = WorldVec * in->GetElement(i * width + j, MEMORYDEVICE_CPU);
        res->GetData(MEMORYDEVICE_CPU)[i * width + j] = WorldVec(2);
      }
    }

    return res;
  }

  ObjSLAM::ObjUChar4Image *ReadOneRGB(std::string Path) {

    ifstream in;

    in.open(Path);
    //read rgb from png file



    auto *res = new ObjSLAM::ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);

    res->ChangeDims(imgSize);

    ReadImageFromFile(res, Path.c_str());

    SaveImageToFile(res, "testRGB");

    return res;
  }

/*  ObjSLAM::ObjFloat4Image* ReadNormal(std::string Path) {

    ifstream in;

    in.open(Path);
    //read rgb from png file


//    ORUtils::Vector2<int> imgSize(width, height);
    auto *res = new ObjSLAM::ObjFloat4Image(imgSize, MEMORYDEVICE_CPU);


    res->ChangeDims(imgSize);

    ReadImageFromFile(res, Path.c_str());


    return res;
  }*/

  std::shared_ptr<ObjSLAM::ObjUIntImage> ReadLabel_OneFile(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<unsigned int> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
//    cout << "**Input size of label is " << vector_in.size() << endl;

    //set dimension
//    ORUtils::Vector2<int> imgSize(width, height);

    //create UintImage object
//    auto *res = new ObjSLAM::ObjUIntImage(imgSize, MEMORYDEVICE_CPU);
    std::shared_ptr<ObjSLAM::ObjUIntImage> res = std::make_shared<ObjSLAM::ObjUIntImage>(imgSize,MEMORYDEVICE_CPU);

    res->ChangeDims(imgSize);


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

  ObjSLAM::LPD_RAW_Pose *ReadLPDRawPose(ifstream &in, double t) {

    /*    ifstream in;
    in.open(Path);*/

    ObjSLAM::LPD_RAW_Pose *res = new ObjSLAM::LPD_RAW_Pose();
    double currentT = 0.0;
    string currentLine;
    double TH = 0.0005;

    while (getline(in, currentLine)) {
      istringstream iss(currentLine);

      iss >> currentT;
//      cout<<"Current T"<<currentT<<endl;

      if (abs(currentT - t) < TH) {
        iss >> res->qw;
        iss >> res->qx;
        iss >> res->qy;
        iss >> res->qz;
//        cout << "read q " << res->qw << res->qx << res->qy << res->qz << endl;
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

  ObjSLAM::ObjShortImage *calculateAffineDFromDepth(ObjSLAM::ObjFloatImage *depth) {

//    ORUtils::Vector2<int> imgSize(width, height);
    auto *disparity = new ObjSLAM::ObjShortImage(imgSize, MEMORYDEVICE_CPU);

    float fx = this->calib->intrinsics_d.projectionParamsSimple.fx;
//    cout<<"fx"<<fx;
    float bxf = 8.0f * this->calib->disparityCalib.GetParams().y * fx;

    for (int y = 0; y < imgSize.y; y++) {
      for (int x = 0; x < imgSize.x; x++) {
        int locId = x + y * imgSize.x;
//
//        float depth_pixel = depth->GetData(MEMORYDEVICE_CPU)[locId];
//        float disparity_tmp = bxf / depth_pixel;
//        short disparity_pixel = this->calib->disparityCalib.GetParams().x - disparity_tmp;

        short short_D_pixel = depth->GetData(MEMORYDEVICE_CPU)[locId] / this->calib->disparityCalib.GetParams().x;

        disparity->GetData(MEMORYDEVICE_CPU)[locId] = short_D_pixel;

      }
    }
    return disparity;
  }

  void setCalib_LPD() {
    calib = new ITMLib::ITMRGBDCalib();

    calib->intrinsics_rgb.SetFrom(640, 480, 320, 320, 320, 240);
    calib->intrinsics_d.SetFrom(640, 480, 320, 320, 320, 240);


    //Calib Extrinsics is between RGB and D, for LPD Dataset it is identity
    ObjSLAM::ObjMatrix4f mat(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    calib->trafo_rgb_to_depth.SetFrom(mat);

    //disparity calib a b physical meanings?
    calib->disparityCalib.SetFrom(/*1135.09*//*0.0002*/1.0,
                                             0.0,
                                             ITMLib::ITMDisparityCalib::TRAFO_AFFINE); //TODO get the values


  }

/*//  void readExtrnsics(string Path){
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
//  }*/



  ITMLib::ITMRGBDCalib *getCalib() {
    return calib;
  }

  void deleteVariables() {

//    delete(this->label_img);
//    delete(this->rgb_img);
//    delete(this->depth_img);
//    delete(this->pose_cw);
//    delete this->label_img;
    delete this->rgb_img;
    delete this->depth_img;
    delete this->pose_wc;
    label_img_vector.clear();
  }

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
    return imgSize;
  }

  ObjSLAM::ObjCameraPose *getPose() { return pose_wc; }

};

#endif //DATASETREADER__LPD_DATASET_H
