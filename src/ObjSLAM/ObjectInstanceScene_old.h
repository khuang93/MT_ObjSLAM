//
// Created by khuang on 7/10/18.
//

#ifndef MT_OBJSLAM_ObjectInstanceScene_old_OLD_H
#define MT_OBJSLAM_ObjectInstanceScene_old_OLD_H


#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMScene.h"


#include "ObjectInstance.h"
#include "ObjectView.h"

#include <vector>
#include <iostream>

namespace ObjSLAM {

//using ObjectVector = std::vector <ObjSLAM::ObjectInstance>;
//access view using pointer because copy constructor supressed
using ViewVector = std::vector <ObjSLAM::ObjectView*>;

//Scene for each single object
template<class TVoxel, class TIndex>
class ObjectInstanceScene_old : public ITMLib::ITMScene<TVoxel, TIndex>{
 private:
//  ObjectVector ListOfAllObjects;
  ViewVector ListofAllViews;
//  ObjectClassLabel label;
//  int objectIndex;


  //TODO
  //Add ObjCameraPose graph and objects graph

 public:

  //Constructor
  ObjectInstanceScene_old(const ITMLib::ITMSceneParams *_sceneParams,
                      bool _useSwapping, MemoryDeviceType _memoryType,  ObjectView* _firstView):
            ITMLib::ITMScene<TVoxel,TIndex>(_sceneParams, _useSwapping, _memoryType){
    ListofAllViews.push_back(_firstView);
    //TODO debug msg
    std::cout << "** Created ObjectInstanceScene_old! \n";
  }



  //Destructor
  ~ObjectInstanceScene_old(){

  }

  //getters


  //setters


  //update

  //adders

};

}

#endif //MT_OBJSLAM_ObjectInstanceScene_old_OLD_H
