//
// Created by khuang on 6/4/18.
//
#include<iostream>
#include"ObjSLAMEngine.h"
#include"test.h"


/// \brief Entry point of ObjSLAM

int main(int argc, char **argv) {
  auto *objSLAM = new ObjSLAM::ObjSLAMEngine;
  //ObjSLAM::ObjSLAMEngine objSLAM;



  delete objSLAM;

  std::cout<<"bla\n";
  return 0;
}
