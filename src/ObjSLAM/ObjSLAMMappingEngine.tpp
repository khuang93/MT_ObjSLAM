//
// Created by khuang on 7/5/18.
//

#include "ObjSLAMMappingEngine.h"


namespace ObjSLAM{

//Constructor with LPD Dataset
ObjSLAMMappingEngine::ObjSLAMMappingEngine(string path, Vector2i _imgSize):imgSize(_imgSize){
  reader = DatasetReader_LPD_Dataset(path, imgSize);

  //process one frame
  reader.readNext();





}




}