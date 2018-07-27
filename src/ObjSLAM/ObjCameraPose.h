//
// Created by khuang on 6/24/18.
//

#ifndef MT_OBJSLAM_POSE_H
#define MT_OBJSLAM_POSE_H

#include <vector>
#include <External/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.h>
#include <External/InfiniTAM/InfiniTAM/ORUtils/Matrix.h>

#include <eigen3/Eigen/Geometry>
//#include "eigen3/Eigen/Sparse"
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "LPD_RAW_Pose.h"

using namespace std;
//using namespace Eigen;

namespace ObjSLAM {

class ObjCameraPose  {



 private:
  Eigen::Quaterniond eigen_pose;
  Eigen::Matrix4d eigen_pose_mat;
//  double tx,ty,tz;
  ORUtils::SE3Pose se3Pose;


 public:
  //Constructor
  ObjCameraPose(double qw, double qx, double qy, double qz, double tx, double ty, double tz);

  ObjCameraPose(ORUtils::SE3Pose _se3pose);

  ObjCameraPose(Eigen::Quaterniond _pose);

  Eigen::Quaterniond getQuaternion();
  Eigen::Matrix4d getEigenMat();

  ORUtils::SE3Pose& getSE3Pose();

  ObjCameraPose& GetTransformationFromPose(ObjCameraPose& otherPose);

  void setQuaternion(double w, double x, double y, double z);

  void setQuaternion(Eigen::Quaterniond _pose);

  friend std::ostream& operator<<(std::ostream& os, ObjCameraPose& obj_cam_pose){
    os<<obj_cam_pose.getSE3Pose().GetM();

    return os;
  }

};

}
#endif //MT_OBJSLAM_POSE_H
