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
  std::cout<<"PCL2IMG\n";
  ORUtils::Matrix3<float> K(this->calib->intrinsics_d.projectionParamsSimple.fx, 0.0f, 0.0f, 0.0f, this->calib->intrinsics_d.projectionParamsSimple.fy, 0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.px,
                            this->calib->intrinsics_d.projectionParamsSimple.py,
                            1.0f);
//  std::cout<<"K"<<K<<std::endl;
  Matrix4f pose_mat = pose.getSE3Pose().GetM();
  std::cout<<pose_mat<<std::endl<<std::endl;


  for(size_t i=0; i < PCL->dataSize; i++){
//    Vector3f point = PCL->GetElement(i,MEMORYDEVICE_CPU).toVector3();
//    Eigen::Vector4f point_eigen(point.x, point.y, point.z, 1);
    Vector4f point = PCL->GetElement(i,MEMORYDEVICE_CPU);



    Vector4f point_camera_frame = pose_mat*point;


    Vector3f pix = K*(point_camera_frame.toVector3());
    Vector2i pix_int (round(pix.x/pix.z), round(pix.y/pix.z));

/*    if(i==0){
      std::cout<<point<<std::endl<<std::endl;
      std::cout<<point_camera_frame<<std::endl<<std::endl;
      std::cout<<pix_int<<std::endl<<std::endl;
    }*/

    if(pix_int.y>=0&&pix_int.x>=0&&pix_int.y<height&&pix_int.x<width){
      int locId = pix_int.y*width+pix_int.x;
      out->GetData(MEMORYDEVICE_CPU)[locId]=pix.z;
    }
  }


}

bool ObjSLAMCamera::projectImg2PointCloud(ObjSLAM::ObjFloatImage *in,
                                          ORUtils::Image<Vector4f> *PCL,
                                          ObjSLAM::ObjCameraPose pose) {
  ORUtils::Matrix4<float > K(this->calib->intrinsics_d.projectionParamsSimple.fx, 0.0f, 0.0f, 0.0f, 0.0f, this->calib->intrinsics_d.projectionParamsSimple.fy, 0.0f, 0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.px,
                            this->calib->intrinsics_d.projectionParamsSimple.py,
                            1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);
  ORUtils::Matrix4<float> K_inv;
  K.inv(K_inv);
  Matrix4f pose_mat = pose.getSE3Pose().GetM();
  Eigen::Matrix4d pose_eig = pose.getEigenMat();
  Eigen::Matrix4d pose_eig_inv = pose_eig.inverse();
  Matrix4f pose_inv;
  pose_mat.inv(pose_inv);
  Matrix4f projMat = pose_inv*K_inv;
  std::cout<<"IMG2PCL\n";

  for(int u= 0; u<imgSize.width;u++){
    for(int v = 0; v<imgSize.height;v++){
      float zc= in->GetData(MEMORYDEVICE_CPU)[v*imgSize.width+u];
      Vector4f pt(u*zc,v*zc,zc,0);
      Vector4f res = projMat*pt;
      int locId=v*width+u;
      PCL->GetData(MEMORYDEVICE_CPU)[locId]=res;
      if(u==1&&v==0){
        std::cout<<pt<<std::endl<<std::endl;
/*
        std::cout<<pose_eig<<std::endl<<std::endl;
        std::cout<<pose_mat<<std::endl<<std::endl;

        std::cout<<pose_eig_inv<<std::endl<<std::endl;
        std::cout<<pose_inv<<std::endl<<std::endl;*/
        std::cout<<res<<std::endl<<std::endl;

      }
    }
  }




}

}