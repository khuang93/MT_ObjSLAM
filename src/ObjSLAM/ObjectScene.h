//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTSCENE_H
#define MT_OBJSLAM_OBJECTSCENE_H

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMScene.h"

#include "ObjectView.h"
#include "ObjectInstance.h"

#include <vector>
#include <iostream>

namespace ObjSLAM {

using ObjectVector = std::vector <ObjSLAM::ObjectInstance>;
using ViewVector = std::vector <ObjSLAM::ObjectView>;

template<class TVoxel, class TIndex>
 class ObjectScene : public ITMLib::ITMScene<TVoxel, TIndex>{
 private:
  ObjectVector* ListOfAllObjects;
  ViewVector* ListofAllViews;

  //TODO
  //Add Pose graph and objects graph

 public:

  //Constructor
  ObjectScene(const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ObjectVector* _objVector, ViewVector* _viewVector):
      ITMLib::ITMScene<TVoxel,TIndex>(_sceneParams, _useSwapping, _memoryType), ListOfAllObjects(_objVector),ListofAllViews(_viewVector){

    //debug msg
    std::cout << "Created Scene! \n";
  }



  //Destructor
  ~ObjectScene(){

  }

  //getters


  //setters


  //update

  //adders

};

}
#endif //MT_OBJSLAM_OBJECTSCENE_H
