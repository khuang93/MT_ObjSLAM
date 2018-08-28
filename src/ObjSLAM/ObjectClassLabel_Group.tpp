//
// Created by khuang on 8/1/18.
//

#include "ObjectClassLabel_Group.h"
//#include "ObjectInstance_New.tpp"

namespace ObjSLAM{

template<typename TVoxel, typename TIndex>
int ObjectClassLabel_Group<TVoxel, TIndex>::getLabelIndex(){
  return LabelIndex;
}
template<typename TVoxel, typename TIndex>
std::string ObjectClassLabel_Group<TVoxel, TIndex>::getLabelClassName(){
  return LabelClassName;
}

template<typename TVoxel, typename TIndex>
void ObjectClassLabel_Group<TVoxel,TIndex>::addObjectInstance(std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> object_ptr){
  this->object_ptr_vector.push_back(object_ptr);
}

template<typename TVoxel, typename TIndex>
std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>> ObjectClassLabel_Group<TVoxel,TIndex>::getObjPtrVector(){
  return object_ptr_vector;
}

/*template<typename TVoxel, typename TIndex>
void ObjectClassLabel_Group<TVoxel,TIndex>::addObjectInstance(ObjectInstance_New<TVoxel, TIndex>* object_ptr){

  std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> ptr(object_ptr);
  this->object_ptr_vector.push_back(ptr);
}*/

}