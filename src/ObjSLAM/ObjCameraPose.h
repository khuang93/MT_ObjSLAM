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

/**
 * @brief Class for camera pose containing eigen pose and the pose from InfiniTAM.
 * unless specified, it saves Twc from World to Cam
 */
class ObjCameraPose  {



 private:
  Eigen::Quaterniond eigen_pose;
  Eigen::Quaterniond eigen_pose_inv;
  Eigen::Matrix4d eigen_pose_mat;
  ORUtils::SE3Pose se3Pose;

  //allocated when needed
  void setAllFromQuaternion(double tx, double ty, double tz);


 public:
  /**
   * @brief Constructor using quaternion and x y z
   * @param qw
   * @param qx
   * @param qy
   * @param qz
   * @param tx
   * @param ty
   * @param tz
   */
  ObjCameraPose(double qw, double qx, double qy, double qz, double tx, double ty, double tz);

  /**
   * @brief Constructor from ORUtils::SE3Pose
   * @param _se3pose
   */
  ObjCameraPose(ORUtils::SE3Pose _se3pose);

  /**
   * @brief Constructor from Eigen::Quaterniond pose
   * @param _pose
   */
  ObjCameraPose(Eigen::Quaterniond _pose);

  /**
   * @brief Getter of the quaternion
   * @return Pose in Eigen::Quaterniond
   */
  Eigen::Quaterniond GetQuaternion();

  /**
   * @brief Getter of the quaternion of the inversed pose
   * @return Inversed pose in Eigen::Quaterniond
   */
  Eigen::Quaterniond GetQuaternionInv();
  
  /**
   * @brief Getter for 4x4 Matrix Pose
   * @return pose in Eigen::Matrix4d
   */
  Eigen::Matrix4d GetEigenMat();

  /**
   * @brief Getter of pose in ORUtils::SE3Pose&
   * @return Pose in ORUtils::SE3Pose&
   */
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
