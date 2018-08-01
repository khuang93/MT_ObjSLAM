//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTSCENE_H
#define MT_OBJSLAM_OBJECTSCENE_H

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMScene.h"

#include "ObjectInstance.h"
#include "ObjectView.h"

#include <vector>
#include <iostream>

namespace ObjSLAM {

using ObjectVector = std::vector <ObjSLAM::ObjectInstance*>;
//access view using pointer because copy constructor supressed
using ViewVector = std::vector <ObjSLAM::ObjectView*>;

//Scene for each single object
template<typename TVoxel, typename TIndex>
class ObjectInstanceScene : public ITMLib::ITMScene<TVoxel, TIndex>{
 private:
//  ObjectVector ListOfAllObjects;
  ViewVector ListofAllViews;
//  ObjectClassLabel label;
//
//  int objectIndex;


  //TODO
  //Add ObjCameraPose graph and objects graph

 public:

  //Constructor
  ObjectInstanceScene(/*ObjectClassLabel _label, int _objectIndex,*/ const ITMLib::ITMSceneParams *_sceneParams,
                      bool _useSwapping, MemoryDeviceType _memoryType,  ObjectView* _firstView);

  /*:
                      label(_label),objectIndex(_objectIndex),
                      ITMLib::ITMScene<TVoxel,TIndex>(_sceneParams, _useSwapping, _memoryType){
    ListofAllViews.push_back(_firstView);
    //TODO debug msg
    std::cout << "** Created ObjectInstanceScene! \n";
  }*/

//  ObjectInstanceScene():label(ObjectClassLabel(0,"0")){  }

//  void setIndex(int idx );
//
//  void setLabel(ObjectClassLabel _label);

  void deleteAll();


  //Destructor
  ~ObjectInstanceScene(){

  }

  //getters


  //setters


  //update

  //adders

};

}
#include "ObjectInstanceScene.tpp"
#endif //MT_OBJSLAM_OBJECTSCENE_H
