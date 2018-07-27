//
// Created by khuang on 7/26/18.
//


#include "ObjSLAMCamera.h"
#include <iostream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <External/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.h>

namespace ObjSLAM{

bool ObjSLAMCamera::projectPointCloud2Img(ORUtils::Image<Vector4f> * PCL, ObjFloatImage* out, ObjCameraPose pose){
/*  Eigen::Matrix3f K; = Eigen::Matrix3d::Zero();
  //K normalized with the focal length in pixels
  K(0, 0) = 1.0f;
  K(1, 1) = 1.0f;
  K(0, 2) = this->calib->intrinsics_d.projectionParamsSimple.px/this->calib->intrinsics_d.projectionParamsSimple.fx;
  K(1, 2) = this->calib->intrinsics_d.projectionParamsSimple.py/this->calib->intrinsics_d.projectionParamsSimple.fy;
  K(2, 2) = 1;

  Eigen::Matrix3f K_inv = K.inverse();
  std::cout<<"K"<<K<<std::endl;*/

  ORUtils::Matrix3<float> K(this->calib->intrinsics_d.projectionParamsSimple.fx, 0.0f, 0.0f, 0.0f, this->calib->intrinsics_d.projectionParamsSimple.fy, 0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.px,
                            this->calib->intrinsics_d.projectionParamsSimple.py,
                            1.0f);
  std::cout<<"K"<<K<<std::endl;



  for(size_t i=0; i < PCL->dataSize; i++){
//    Vector3f point = PCL->GetElement(i,MEMORYDEVICE_CPU).toVector3();
//    Eigen::Vector4f point_eigen(point.x, point.y, point.z, 1);
    Vector4f point = PCL->GetElement(i,MEMORYDEVICE_CPU);

    Matrix4f pose_mat = pose.getSE3Pose().GetM();

    Vector4f point_camera_frame = pose_mat*point;


    Vector3f pix = K*(point_camera_frame.toVector3());
    Vector2i pix_int (round(pix.x/pix.z), round(pix.y/pix.z));
    if(i>250+640*400&&i<350+640*400){
      std::cout<<point<<std::endl<<std::endl;
      std::cout<<point_camera_frame<<std::endl<<std::endl;
      std::cout<<pix_int<<std::endl<<std::endl;
    }
  }


}

}