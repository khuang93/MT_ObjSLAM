//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTSCENE_H
#define MT_OBJSLAM_OBJECTSCENE_H

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMScene.h"


#include "ObjectInstance.h"
#include "ObjectView_New.h"

#include <vector>
#include <iostream>

namespace ObjSLAM {

using ObjectVector = std::vector <ObjSLAM::ObjectInstance>;
//access view using pointer because copy constructor supressed
using ViewVector = std::vector <ObjSLAM::ObjectView_New*>;

//Scene for each single object
template<class TVoxel, class TIndex>
 class ObjectInstanceScene : public ITMLib::ITMScene<TVoxel, TIndex>{
 private:
//  ObjectVector ListOfAllObjects;
  ViewVector ListofAllViews;

  //TODO
  //Add ObjCameraPose graph and objects graph

 public:

  //Constructor
  ObjectInstanceScene(const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType,  ObjectView_New* _firstView):
      ITMLib::ITMScene<TVoxel,TIndex>(_sceneParams, _useSwapping, _memoryType){
   ListofAllViews.push_back(_firstView);
    //TODO debug msg
    std::cout << "** Created ObjectInstanceScene! \n";
  }



  //Destructor
  ~ObjectInstanceScene(){

  }

  //getters


  //setters


  //update

  //adders

};

}
#endif //MT_OBJSLAM_OBJECTSCENE_H
