//
// Created by khuang on 6/11/18.
//

#ifndef OBJSLAMMAPPER_DATASETREADER_H
#define OBJSLAMMAPPER_DATASETREADER_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
//#include <opencv2/opencv.hpp>
#include "ObjSLAMDataTypes.h"
#include <stdlib.h>


#include "../../External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"

using namespace std;

class DatasetReader_LPD_Dataset {
 private:
  int width, height;

 public:
  DatasetReader_LPD_Dataset() {};

  DatasetReader_LPD_Dataset(int w, int h) : width(w), height(h) {};

  void setWidth(int w) {
    width = w;
  }
  void setHeight(int h) {
    height = h;
  }
  int getWidth() {
    return width;
  }
  int getHeight() {
    return height;
  }


  ObjSLAM::ObjFloatImage* ReadDepth(std::string Path) {
    ifstream in;

    in.open(Path);

    vector<float> vector_in;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    //TODO debug output
    cout << "**Input size is "<<vector_in.size()<<endl;

    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjFloatImage(newDims, MEMORYDEVICE_CPU);

    res->ChangeDims(newDims);


//
    for (int i = 0; i < width; i++) {
//      cout<<"i"<<i<<endl;
      for (int j = 0; j < height; j++) {
//        res[height * i + j] = vector_in.at(height * i + j);

        res->GetData(MEMORYDEVICE_CPU)[height * i + j] = vector_in.at(height * i + j);
      }

    }
    return res;

  }

  ObjSLAM::ObjUChar4Image* ReadRGB(std::string Path) {

    ifstream in;

    in.open(Path);
  //read rgb from png file


    ORUtils::Vector2<int> newDims(width, height);
    auto *res = new ObjSLAM::ObjUChar4Image(newDims, MEMORYDEVICE_CPU);


    res->ChangeDims(newDims);

//    ReadImageFromFile(res, Path.c_str());


    return res;

//  return nullptr;
  }
};

#endif //OBJSLAMMAPPER_DATASETREADER_H
