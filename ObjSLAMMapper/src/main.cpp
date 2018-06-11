//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>

#include "DatasetReader.h"

using namespace std;

Eigen::MatrixXf ReadDepth (std::string Path){
  ifstream in;

  in.open(Path);

  vector<float> readin;

  while(in.peek()!=EOF){
    float tmp;
    in>>tmp;
    readin.push_back(tmp);
  }

  cout<<readin.size();

  Eigen::MatrixXf res(640,480);
  for(int i = 0; i<640;i++){
    for(int j = 0; j<480;j++){
//      cout << 480*i+j<<endl;
      res(i,j)=readin.at(480*i+j);
    }
  }
  return res;

}

int main(int argc, char** argv){

  cout<<"Hello World"<<endl;

  string path = argv[1];

  Eigen::MatrixXf img;

  img = DatasetReader::ReadDepth(path);

  cout <<img(0,0);
  return 0;
}
