#include "ObjSLAMEngine.h"

// Constructors/Destructors
//  
namespace ObjSLAM {
ObjSLAMEngine::ObjSLAMEngine() {
  initAttributes();
}

ObjSLAMEngine::~ObjSLAMEngine() {}

//  
// Methods
//  
ObjSLAM::InfiniTAMEngine init_InfiniTAMCreator (){
  infiniTAMCreator = new InfiniTAMEngine;

}

// Accessor methods
//  


// Other methods
//  

void ObjSLAMEngine::initAttributes() {
  std::cout << "initAttr\n";
}

}