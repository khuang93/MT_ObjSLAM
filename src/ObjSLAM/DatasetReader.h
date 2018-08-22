//
// Created by khuang on 8/22/18.
//

#ifndef MT_OBJSLAM_DATASETREADER_H
#define MT_OBJSLAM_DATASETREADER_H

#endif //MT_OBJSLAM_DATASETREADER_H

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

class DatasetReader {

};