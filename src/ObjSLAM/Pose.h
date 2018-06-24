//
// Created by khuang on 6/24/18.
//

#ifndef MT_OBJSLAM_POSE_H
#define MT_OBJSLAM_POSE_H

#include <vector>

#include "eigen3/Eigen/Geometry"
#include "LPD_RAW_Pose.h"

using namespace std;

namespace ObjSLAM {

class Pose {



 private:
  Eigen::Quaterniond eigen_pose;
  LPD_RAW_Pose LPD_pose;


 public:
  Pose(double w, double x, double y, double z):eigen_pose(w,x,y,z){}

  Pose(Eigen::Quaterniond _pose):eigen_pose(_pose){}


  Eigen::Quaterniond getQuaternion();

  void setQuaternion(double w, double x, double y, double z);

  void setQuaternion(Eigen::Quaterniond _pose);




};

}
#endif //MT_OBJSLAM_POSE_H
