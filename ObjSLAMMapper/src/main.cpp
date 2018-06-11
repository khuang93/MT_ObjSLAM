//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>

#include "DatasetReader_LPD_Dataset.h"

using namespace std;

int main(int argc, char** argv){

  cout<<"Hello World"<<endl;

  string path = argv[1];

  Eigen::MatrixXf img;

  DatasetReader_LPD_Dataset reader(640,480);
  cout<<reader.getHeight()<<" "<<reader.getWidth()<<endl;
  

  img = reader.ReadDepth(path);

  cout <<img(0,0);
  return 0;
}
