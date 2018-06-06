//
// Created by khuang on 6/6/18.
//
#include <iostream>
#include "ObjSLAMEngine.h"


///@brief Entry point of ObjSLAM

int main(int argc, char ** argv) {
  auto *objSLAM = new ObjSLAM::ObjSLAMEngine();




  delete objSLAM;

  return 0;
}
