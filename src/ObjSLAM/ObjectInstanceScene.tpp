//
// Created by khuang on 6/6/18.
//
#pragma once
#include "ObjectInstanceScene.h"


namespace ObjSLAM{


template <typename TVoxel, typename TIndex>
ObjectInstanceScene<TVoxel, TIndex>::ObjectInstanceScene(ObjSLAM::ObjectClassLabel _label,
                                                         int _objectIndex,
                                                         const ITMLib::ITMSceneParams *_sceneParams,
                                                         bool _useSwapping,
                                                         MemoryDeviceType _memoryType,
                                                         ObjSLAM::ObjectView_New *_firstView):
label(_label),objectIndex(_objectIndex),ITMLib::ITMScene<TVoxel,TIndex>(_sceneParams, _useSwapping, _memoryType){
  ListofAllViews.push_back(_firstView);
  //TODO debug msg
  std::cout << "** Created ObjectInstanceScene! \n";
}


template <typename TVoxel, typename TIndex>
void ObjectInstanceScene<TVoxel, TIndex>::deleteAll() {

}

}