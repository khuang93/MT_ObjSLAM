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

//unless specified, it saves Twc from World to Cam
class ObjCameraPose  {



 private:
  Eigen::Quaterniond eigen_pose;
  Eigen::Quaterniond eigen_pose_inv;
  Eigen::Matrix4d eigen_pose_mat;
//  double tx,ty,tz;
  ORUtils::SE3Pose se3Pose;

  //allocated when needed
  void setAllFromQuaternion(double tx, double ty, double tz);


 public:
  //Constructor
  ObjCameraPose(double qw, double qx, double qy, double qz, double tx, double ty, double tz);

  ObjCameraPose(ORUtils::SE3Pose _se3pose);

  ObjCameraPose(Eigen::Quaterniond _pose);



  Eigen::Quaterniond GetQuaternion();

  Eigen::Quaterniond GetQuaternionInv();
  
  Eigen::Matrix4d GetEigenMat();

  ORUtils::SE3Pose& GetSE3Pose();

  static ObjCameraPose GetTransformation(ObjCameraPose& fromPose, ObjCameraPose& toPose);

  static ObjCameraPose GetTransformation(ORUtils::SE3Pose& fromPose, ORUtils::SE3Pose& toPose);

  void SetQuaternion(double w, double x, double y, double z);

  void SetQuaternion(Eigen::Quaterniond _pose);

  friend std::ostream& operator<<(std::ostream& os, ObjCameraPose& obj_cam_pose){
    os<< obj_cam_pose.GetSE3Pose().GetM();

    return os;
  }




};

}
#endif //MT_OBJSLAM_POSE_H
