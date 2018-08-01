//
// Created by khuang on 7/9/18.
//

#include "IOUtils.h"



void IOUtils::write2PLYfile(const std::vector<Eigen::Vector3f>& pcl, const std::string filename)
{
  std::ofstream fileWriter;
  fileWriter.open(filename.c_str(), std::ofstream::out/* | std::ofstream::app*/);
  if(!fileWriter.is_open())
  {
    std::cout << "Cannot open file for PLY file writing " << filename.c_str() << "\n";
  }
  std::cout << "Point cloud is saved to " << filename.c_str() << "\n";

  // write header
  fileWriter << "ply\n";
  fileWriter << "format ascii 1.0\n";
  fileWriter << "element vertex " << pcl.size() << "\n";
  fileWriter << "property float32 x\n";
  fileWriter << "property float32 y\n";
  fileWriter << "property float32 z\n";
  fileWriter << "end_header\n";

  for (int i = 0; i < pcl.size(); i++)
  {
    Eigen::Vector3f point = pcl.at(i);
    fileWriter << point(0, 0) << " " << point(1, 0) << " " << point(2, 0) << "\n";
  }

  fileWriter.close();
}

/*
static std::vector<Eigen::Vector3f>& IOUtils::convertDepthToPointCloud(const ObjSLAM::ObjFloatImage* depth){

}*/
