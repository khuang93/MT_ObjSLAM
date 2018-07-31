//
// Created by khuang on 6/24/18.
//

#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMMath.h>
#include "ObjCameraPose.h"

namespace ObjSLAM {

ObjCameraPose::ObjCameraPose(ORUtils::SE3Pose _se3pose) : se3Pose(_se3pose) {

  Eigen::Matrix3d eigen_mat;
  eigen_mat(0, 0) = se3Pose.GetR().m00;
  eigen_mat(0, 1) = se3Pose.GetR().m10;
  eigen_mat(0, 2) = se3Pose.GetR().m20;
  eigen_mat(1, 0) = se3Pose.GetR().m01;
  eigen_mat(1, 1) = se3Pose.GetR().m11;
  eigen_mat(1, 2) = se3Pose.GetR().m21;
  eigen_mat(2, 0) = se3Pose.GetR().m02;
  eigen_mat(2, 1) = se3Pose.GetR().m12;
  eigen_mat(2, 2) = se3Pose.GetR().m22;
  ORUtils::Vector3<float> t = se3Pose.GetT();

  eigen_pose = Eigen::Quaterniond(eigen_mat);
  Eigen::Affine3d rot(eigen_pose.normalized().toRotationMatrix());
  Eigen::Affine3d trans(Eigen::Translation3d(t.x,t.y,t.z));
  eigen_pose_mat = (rot*trans).matrix();

//    cout<<"**DEBUG: SE3Mat\n"<<this->getSE3Pose().GetM()<<endl;
//    //TODO Debug msg
//    cout<<"ObjCameraPose from SE3Pose created\n";
}

ObjCameraPose::ObjCameraPose(double qw, double qx, double qy, double qz, double tx, double ty, double tz) : eigen_pose(
    qw,
    qx,
    qy,
    qz) {
  setAllFromQuaternion(tx, ty, tz);
}

ObjCameraPose::ObjCameraPose(Eigen::Quaterniond _pose) : eigen_pose(_pose) {
  setAllFromQuaternion(0,0,0);
}

void ObjCameraPose::setAllFromQuaternion(double tx, double ty, double tz) {
  Eigen::Matrix3d eigen_mat = eigen_pose.normalized().toRotationMatrix();

  Eigen::Affine3d rot(eigen_pose.normalized().toRotationMatrix());
  Eigen::Affine3d trans(Eigen::Translation3d(tx,ty,tz));
  eigen_pose_mat = (rot*trans).matrix();

  double m00 = eigen_pose_mat(0, 0);
  double m01 = eigen_pose_mat(0, 1);
  double m02 = eigen_pose_mat(0, 2);
  double m10 = eigen_pose_mat(1, 0);
  double m11 = eigen_pose_mat(1, 1);
  double m12 = eigen_pose_mat(1, 2);
  double m20 = eigen_pose_mat(2, 0);
  double m21 = eigen_pose_mat(2, 1);
  double m22 = eigen_pose_mat(2, 2);

  ORUtils::Matrix3<float> OR_mat(m00, m10, m20, m01, m11, m21, m02, m12, m22);
  ORUtils::Vector3<float> OR_vec(tx, ty, tz);

  auto *_pose = new ORUtils::SE3Pose(OR_mat, OR_mat*OR_vec);
//    cout<<"**DEBUG: Mat"<<_pose->GetM()<<endl;
  se3Pose = *_pose;
  delete _pose;
  //TODO Debug msg
  cout << "ObjCameraPose from Quaternion created\n";
}

Eigen::Quaterniond ObjCameraPose::getQuaternion() {
  return eigen_pose;
}


ObjCameraPose ObjCameraPose::GetTransformation(ObjCameraPose& fromPose, ObjCameraPose& toPose){
  ORUtils::Matrix4<float> fromMatrix = fromPose.getSE3Pose().GetM();
  ORUtils::Matrix4<float> toMatrix = toPose.getSE3Pose().GetM();
  ORUtils::Matrix4<float>  inv_fromMatrix;
  fromMatrix.inv(inv_fromMatrix);
  ORUtils::Matrix4<float> resMatrix = toMatrix*inv_fromMatrix;
  ORUtils::SE3Pose resPose(resMatrix);
  return ObjCameraPose(resPose);
}

ObjCameraPose ObjCameraPose::GetTransformation(ORUtils::SE3Pose& fromPose, ORUtils::SE3Pose& toPose){
  ORUtils::Matrix4<float> fromMatrix = fromPose.GetM();
  ORUtils::Matrix4<float> toMatrix = toPose.GetM();
  ORUtils::Matrix4<float>  inv_fromMatrix;
  fromMatrix.inv(inv_fromMatrix);
  ORUtils::Matrix4<float> resMatrix = toMatrix*inv_fromMatrix;
  ORUtils::SE3Pose resPose(resMatrix);
  return ObjCameraPose(resPose);
}


Eigen::Matrix4d ObjCameraPose::getEigenMat() {
  return eigen_pose_mat;
}

ORUtils::SE3Pose &ObjCameraPose::getSE3Pose() {
  return se3Pose;
}

void ObjCameraPose::setQuaternion(double w, double x, double y, double z) {
  Eigen::Quaterniond temp;
  temp.w() = w;
  temp.x() = x;
  temp.y() = y;
  temp.z() = z;
  eigen_pose = temp;
}

void ObjCameraPose::setQuaternion(Eigen::Quaterniond _pose) {
  eigen_pose = _pose;
}

/*void ObjCameraPose::setCamera(const ObjSLAMCamera* _cam){
  camera=_cam;
}*/



}