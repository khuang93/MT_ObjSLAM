//
// Created by khuang on 8/1/18.
//

#include "ObjectInstance_New.h"

namespace ObjSLAM{


template<typename TVoxel, typename TIndex>
void ObjectInstance_New<TVoxel, TIndex>::setScene(std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> _scene){
  scene=_scene;
}

template<typename TVoxel, typename TIndex>
void ObjectInstance_New<TVoxel, TIndex>::setAnchorView(std::shared_ptr<ObjectView_New<TVoxel,TIndex>> _anchor_view){
  anchor_view=_anchor_view;
}

template<typename TVoxel, typename TIndex>
std::shared_ptr<ObjectView_New<TVoxel,TIndex>> ObjectInstance_New<TVoxel, TIndex>::getAnchorView(){
  return anchor_view;
};


template<typename TVoxel, typename TIndex>
std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> ObjectInstance_New<TVoxel, TIndex>::getScene(){
  return scene;
}

template<typename TVoxel, typename TIndex>
std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> ObjectInstance_New<TVoxel, TIndex>::getClassLabel(){
  return label;
};


}