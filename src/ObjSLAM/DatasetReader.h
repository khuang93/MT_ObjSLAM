//
// Created by khuang on 8/22/18.
//

#ifndef MT_OBJSLAM_DATASETREADER_H
#define MT_OBJSLAM_DATASETREADER_H



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
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <src/ObjSLAM/ObjSLAMDataTypes.h>

using namespace std;

/** \brief
	    Basic interface to read any kind of data sets.
	*/
class DatasetReader {
 protected:
  int width, height;
  Vector2i imgSize;
//  ITMLib::ITMRGBDCalib *calib = nullptr;
  std::shared_ptr<ITMLib::ITMRGBDCalib> calib;
  string path;
  int img_number = 1;
  ITMLib::ITMViewBuilder *viewBuilder = nullptr;
  std::vector<std::string> LabelFileNames;

 public:
  std::shared_ptr<ObjSLAM::ObjUChar4Image> rgb_img;
    std::shared_ptr<ObjSLAM::ObjFloatImage> depth_img;
  std::shared_ptr<ObjSLAM::ObjUIntImage> label_img;
  LabelImgVector label_img_vector;

  DatasetReader(string _path, Vector2i _imgSize):path(_path),imgSize(_imgSize){
    width = imgSize.x;
    height = imgSize.y;
    string label_path = path + "/pixel_label/";
    GetFileNames(label_path);
  }

  /** Virtual function, reads the next frame and returns the frame number as int.
		*/
  virtual int ReadNext()=0;

//  virtual ObjSLAM::ObjShortImage *ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth)=0;

//  virtual bool ReadCalib()=0;

  std::vector<std::string> GetFileNames(std::string directoryPath);

  virtual std::shared_ptr<ObjSLAM::ObjFloatImage> ReadOneDepth(std::string Path);

  virtual std::shared_ptr<ObjSLAM::ObjShortImage> ReadOneDisparity(std::string Path);

  std::shared_ptr<ObjSLAM::ObjUChar4Image> ReadOneRGB(std::string Path);

  std::shared_ptr<ObjSLAM::ObjUIntImage> ReadLabel_OneFile(std::string Path);

//  ITMLib::ITMRGBDCalib *GetCalib();
  std::shared_ptr<ITMLib::ITMRGBDCalib> GetCalib();

  void setWidth(int w);
  void setHeight(int h);
  int getWidth();
  int getHeight();
  Vector2i GetSize();

  virtual bool ReadCalib(string calib_path);

  virtual ~DatasetReader(){
    delete viewBuilder;
  }
};
#endif //MT_OBJSLAM_DATASETREADER_H