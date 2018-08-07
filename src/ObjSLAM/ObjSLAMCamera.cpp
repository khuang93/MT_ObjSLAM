//
// Created by khuang on 7/26/18.
//


#include "ObjSLAMCamera.h"

namespace ObjSLAM {

bool ObjSLAMCamera::projectPointCloud2Img(ORUtils::Image<Vector4f> *PCL, ObjFloatImage *out, ObjCameraPose pose) {
/*  Eigen::Matrix3f K; = Eigen::Matrix3d::Zero();
  //K normalized with the focal length in pixels
  K(0, 0) = 1.0f;
  K(1, 1) = 1.0f;
  K(0, 2) = this->calib->intrinsics_d.projectionParamsSimple.px/this->calib->intrinsics_d.projectionParamsSimple.fx;
  K(1, 2) = this->calib->intrinsics_d.projectionParamsSimple.py/this->calib->intrinsics_d.projectionParamsSimple.fy;
  K(2, 2) = 1;

  Eigen::Matrix3f K_inv = K.inverse();
  std::cout<<"K"<<K<<std::endl;*/
//  std::cout << "PCL2IMG\n";
  ORUtils::Matrix3<float> K(this->calib->intrinsics_d.projectionParamsSimple.fx,
                            0.0f,
                            0.0f,
                            0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.fy,
                            0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.px,
                            this->calib->intrinsics_d.projectionParamsSimple.py,
                            1.0f);
//  std::cout<<"K"<<K<<std::endl;
  Matrix4f pose_mat = pose.getSE3Pose().GetM();
//  std::cout<<pose_mat<<std::endl<<std::endl;


  for (size_t i = 0; i < PCL->dataSize; i++) {
//    Vector3f point = PCL->GetElement(i,MEMORYDEVICE_CPU).toVector3();
//    Eigen::Vector4f point_eigen(point.x, point.y, point.z, 1);
    Vector4f point = PCL->GetElement(i, MEMORYDEVICE_CPU);

    Vector4f point_camera_frame = pose_mat * point;

    Vector3f pix = K * (point_camera_frame.toVector3());
    Vector2i pix_int(round(pix.x / pix.z), round(pix.y / pix.z));

    if (pix_int.y >= 0 && pix_int.x >= 0 && pix_int.y < height && pix_int.x < width) {
      int locId = pix_int.y * width + pix_int.x;
      out->GetData(MEMORYDEVICE_CPU)[locId] = pix.z;
    }
  }

}

//return value: min_xyz and then max_xyz
ORUtils::Vector6<float> ObjSLAMCamera::projectImg2PointCloud(ObjSLAM::ObjFloatImage *in,
                                          ORUtils::Image<Vector4f> *PCL,
                                          ObjSLAM::ObjCameraPose pose) {
  ORUtils::Matrix4<float> K(this->calib->intrinsics_d.projectionParamsSimple.fx,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.fy,
                            0.0f,
                            0.0f,
                            this->calib->intrinsics_d.projectionParamsSimple.px,
                            this->calib->intrinsics_d.projectionParamsSimple.py,
                            1.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            1.0f);
  ORUtils::Matrix4<float> K_inv;
  K.inv(K_inv);
  Matrix4f pose_mat = pose.getSE3Pose().GetM();
  Eigen::Matrix4d pose_eig = pose.getEigenMat();
  Eigen::Matrix4d pose_eig_inv = pose_eig.inverse();
  Matrix4f pose_inv;
  pose_mat.inv(pose_inv);
//  Matrix4f projMat = pose_inv*K_inv;

  Matrix4f projMat;
  (K * pose_mat).inv(projMat);

//  std::cout << "IMG2PCL\n";
  double min_x=10000.0;
  double min_y=min_x;
  double min_z=min_x;
  double max_x=0.0;
  double max_y=max_x;
  double max_z=max_x;
  for (int v = 0; v < imgSize.height; v++) {
    for (int u = 0; u < imgSize.width; u++) {
      int locId = v * imgSize.width + u;

      float zc = in->GetData(MEMORYDEVICE_CPU)[locId];
      Vector4f pt(u * zc, v * zc, zc, 1);
      Vector4f res = projMat * pt;
      if(res.x>max_x) max_x=res.x;
      if(res.x<min_x) min_x=res.x;
      if(res.y>max_y) max_y=res.y;
      if(res.y<min_y) min_y=res.y;
      if(res.z>max_z) max_z=res.z;
      if(res.z<min_z) min_z=res.z;
      PCL->GetData(MEMORYDEVICE_CPU)[locId] = res;
    }
  }
  ORUtils::Vector6<float> boundingCube(min_x,min_y,min_z,max_x,max_y,max_z);
//  cout<<"min xyz="<<min_x<<" "<<min_y<<" "<<min_z<<endl;
//  cout<<"max xyz="<<max_x<<" "<<max_y<<" "<<max_z<<endl;
  return boundingCube;
}

}