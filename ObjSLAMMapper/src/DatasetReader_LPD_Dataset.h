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

using namespace std;

class DatasetReader_LPD_Dataset {
 private:
  int width ,height;


 public:
  DatasetReader_LPD_Dataset(){

  };

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

};

/*static*/ Eigen::MatrixXd ReadRGB (std::string path){

}

#endif //OBJSLAMMAPPER_DATASETREADER_H
