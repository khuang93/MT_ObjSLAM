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

using namespace std;

using LabelImgVector = std::vector<std::shared_ptr<ObjSLAM::ObjUIntImage>>;

/** \brief
	    Basic interface to read any kind of data sets.
	*/
class DatasetReader {
 protected:
  int width, height;
  Vector2i imgSize;
  ITMLib::ITMRGBDCalib *calib = nullptr;
  string path;
  int img_number = 1;
  ITMLib::ITMViewBuilder *viewBuilder = nullptr;
 public:
  ObjSLAM::ObjUChar4Image *rgb_img;
  ObjSLAM::ObjFloatImage *depth_img;
  std::shared_ptr<ObjSLAM::ObjUIntImage> label_img;
  LabelImgVector label_img_vector;

  DatasetReader(string _path, Vector2i _imgSize):path(_path),imgSize(_imgSize){
    width = imgSize.x;
    height = imgSize.y;
  }

  /** Virtual function, reads the next frame and returns the frame number as int.
		*/
  virtual int readNext()=0;

//  virtual ObjSLAM::ObjShortImage *ConvertToRealDepth(ObjSLAM::ObjFloatImage *depth)=0;

//  virtual bool readCalib()=0;

  std::vector<std::string> getFileNames(std::string directoryPath);

  virtual ObjSLAM::ObjFloatImage *ReadOneDepth(std::string Path);

  virtual ObjSLAM::ObjShortImage *ReadOneDisparity(std::string Path);

  ObjSLAM::ObjUChar4Image *ReadOneRGB(std::string Path);

  std::shared_ptr<ObjSLAM::ObjUIntImage> ReadLabel_OneFile(std::string Path);

  ITMLib::ITMRGBDCalib *getCalib();

  void setWidth(int w);
  void setHeight(int h);
  int getWidth();
  int getHeight();
  Vector2i getSize();

  virtual bool readCalib(string calib_path);

  virtual ~DatasetReader(){
    delete viewBuilder;
  }
};
#endif //MT_OBJSLAM_DATASETREADER_H