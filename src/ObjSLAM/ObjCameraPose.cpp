//
// Created by khuang on 6/24/18.
//

#include "ObjCameraPose.h"


namespace ObjSLAM{

ObjCameraPose::ObjCameraPose(ORUtils::SE3Pose _se3pose):se3Pose(_se3pose) {


    Eigen::Matrix3d eigen_mat;
    eigen_mat(0,0) = se3Pose.GetR().m00;eigen_mat(0,1) = se3Pose.GetR().m10;eigen_mat(0,2) = se3Pose.GetR().m20;
    eigen_mat(1,0) = se3Pose.GetR().m01;eigen_mat(1,1) = se3Pose.GetR().m11;eigen_mat(1,2) = se3Pose.GetR().m21;
    eigen_mat(2,0) = se3Pose.GetR().m02;eigen_mat(2,1) = se3Pose.GetR().m12;eigen_mat(2,2) = se3Pose.GetR().m22;

    eigen_pose=Eigen::Quaterniond(eigen_mat);
//    cout<<"**DEBUG: SE3Mat\n"<<this->getSE3Pose().GetM()<<endl;
//    //TODO Debug msg
//    cout<<"ObjCameraPose from SE3Pose created\n";
}

ObjCameraPose::ObjCameraPose(double qw, double qx, double qy, double qz, double tx, double ty, double tz) :eigen_pose(qw,qx,qy,qz){
    Eigen::Matrix3d eigen_mat = eigen_pose.normalized().toRotationMatrix();
    double m00 = eigen_mat(0,0);    double m01 = eigen_mat(0,1);    double m02 = eigen_mat(0,2);
    double m10 = eigen_mat(1,0);double m11 = eigen_mat(1,1);double m12 = eigen_mat(1,2);
    double m20 = eigen_mat(2,0);double m21 = eigen_mat(2,1);double m22 = eigen_mat(2,2);

    ORUtils::Matrix3<float > OR_mat(m00,m10,m20,m01,m11,m21,m02,m12,m22);
    ORUtils::Vector3<float > OR_vec(tx,ty,tz);

    auto* _pose = new ORUtils::SE3Pose(OR_mat, OR_vec);
//    cout<<"**DEBUG: Mat"<<_pose->GetM()<<endl;
    se3Pose=*_pose;
    delete _pose;
    //TODO Debug msg
    cout<<"ObjCameraPose from Quaternion created\n";
}

ObjCameraPose::ObjCameraPose(Eigen::Quaterniond _pose) :eigen_pose(_pose){}

Eigen::Quaterniond ObjCameraPose::getQuaternion(){
    return eigen_pose;
}


ORUtils::SE3Pose& ObjCameraPose::getSE3Pose() {
    return se3Pose;
}

void ObjCameraPose::setQuaternion(double w, double x, double y, double z){
    Eigen::Quaterniond temp(w,x,y,z);

    eigen_pose=temp;
}

void ObjCameraPose::setQuaternion(Eigen::Quaterniond _pose){
    eigen_pose=_pose;
}


}