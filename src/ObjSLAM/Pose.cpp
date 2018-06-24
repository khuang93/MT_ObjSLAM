//
// Created by khuang on 6/24/18.
//

#include "Pose.h"


namespace ObjSLAM{

Eigen::Quaterniond Pose::getQuaternion(){
    return eigen_pose;
}

void Pose::setQuaternion(double w, double x, double y, double z){
    Eigen::Quaterniond temp(w,x,y,z);

    eigen_pose=temp;
}

void Pose::setQuaternion(Eigen::Quaterniond _pose){
    eigen_pose=_pose;
}


}