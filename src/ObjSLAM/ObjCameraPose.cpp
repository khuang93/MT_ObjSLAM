//
// Created by khuang on 6/24/18.
//

#include "ObjCameraPose.h"


namespace ObjSLAM{

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