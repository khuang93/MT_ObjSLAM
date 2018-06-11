//
// Created by khuang on 6/11/18.
//

#ifndef OBJSLAMMAPPER_DATASETREADER_H
#define OBJSLAMMAPPER_DATASETREADER_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ObjSLAMDataTypes.h"

using namespace std;

class DatasetReader_LPD_Dataset {
 private:
  int width ,height;


 public:
  DatasetReader_LPD_Dataset(){};

  DatasetReader_LPD_Dataset(int w, int h):width(w),height(h){};


  void setWidth(int w){
    width=w;
  }
  void setHeight(int h){
    height=h;
  }
  int getWidth(){
    return width;
  }
  int getHeight(){
    return height;
  }

  /*static*/ Eigen::MatrixXf ReadDepth (std::string Path) {
    ifstream in;

    in.open(Path);

    vector<float> readin;

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      readin.push_back(tmp);
    }

    cout << readin.size();

    Eigen::MatrixXf res(width, height);
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        res(i, j) = readin.at(height * i + j);
      }
    }
    return res;
  }
  /*static*/ Eigen::MatrixXf ReadDepth (std::string Path, std::string Name) {
    ifstream in;

    in.open(Path+Name);

    vector<float> vector_in;
    

    while (in.peek() != EOF) {
      float tmp;
      in >> tmp;
      vector_in.push_back(tmp);
    }

    cout << vector_in.size();

    Eigen::MatrixXf res(width, height);
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        res(i, j) = vector_in.at(height * i + j);
      }
    }
    return res;
  }

};

/*static*/ Eigen::MatrixXd ReadRGB (std::string path){

}

#endif //OBJSLAMMAPPER_DATASETREADER_H
