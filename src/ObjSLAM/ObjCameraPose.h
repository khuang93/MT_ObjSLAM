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
//  double tx,ty,tz;
  ORUtils::SE3Pose se3Pose;


 public:
  //Constructor
  ObjCameraPose(double qw, double qx, double qy, double qz, double tx, double ty, double tz):eigen_pose(qw,qx,qy,qz){
    Eigen::Matrix3d eigen_mat = eigen_pose.normalized().toRotationMatrix();
    double m00 = eigen_mat(0,0);    double m01 = eigen_mat(0,1);    double m02 = eigen_mat(0,2);
    double m10 = eigen_mat(1,0);double m11 = eigen_mat(1,1);double m12 = eigen_mat(1,2);
    double m20 = eigen_mat(2,0);double m21 = eigen_mat(2,1);double m22 = eigen_mat(2,2);

    ORUtils::Matrix3<float > OR_mat(m00,m01,m02,m10,m11,m12,m20,m21,m22);
    ORUtils::Vector3<float > OR_vec(tx,ty,tz);


    ORUtils::SE3Pose _pose(OR_mat, OR_vec);

    se3Pose=_pose;

    //TODO Debug msg
    cout<<"ObjCameraPose created\n";
  }

  ObjCameraPose(Eigen::Quaterniond _pose):eigen_pose(_pose){}


  Eigen::Quaterniond getQuaternion();

  ORUtils::SE3Pose* getSE3Pose();

  void setQuaternion(double w, double x, double y, double z);

  void setQuaternion(Eigen::Quaterniond _pose);




};

}
#endif //MT_OBJSLAM_POSE_H
