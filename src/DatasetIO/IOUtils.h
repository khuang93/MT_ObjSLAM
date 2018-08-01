//
// Created by khuang on 7/9/18.
//

#ifndef MT_OBJSLAM_IOUTILS_H
#define MT_OBJSLAM_IOUTILS_H
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include "src/ObjSLAM/ObjSLAMDataTypes.h"
//#include <eigen3/Eigen/src/Core/Matrix.h>

class IOUtils {

   void write2PLYfile(const std::vector<Eigen::Vector3f>& pcl, const std::string filename);

//  static std::vector<Eigen::Vector3f>& convertDepthToPointCloud(const ObjSLAM::ObjFloatImage* depth);

};

#endif //MT_OBJSLAM_IOUTILS_H
