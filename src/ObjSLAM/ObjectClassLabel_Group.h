//
// Created by khuang on 8/1/18.
//

#ifndef MT_OBJSLAM_OBJECTCLASSLABEL_GROUP_H
#define MT_OBJSLAM_OBJECTCLASSLABEL_GROUP_H
#include <string>
#include <memory>
#include <vector>
namespace ObjSLAM{

template<typename TVoxel, typename TIndex>
class ObjectInstance_New;


template<typename TVoxel, typename TIndex>
class ObjectClassLabel_Group {
 private:
  int LabelIndex;
  std::string LabelClassName;
  std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>> object_ptr_vector;

  public:
  ObjectClassLabel_Group(int _index, std::string _labelClassName):LabelIndex(_index), LabelClassName(_labelClassName){}

  int getLabelIndex(){  return LabelIndex;}

  std::string getLabelClassName(){  return LabelClassName;}

  void addObjectInstance(std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> object_ptr){this->object_ptr_vector.push_back(object_ptr);}

  std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>>& getObjPtrVector(){return object_ptr_vector;}


  friend std::ostream& operator<< (std::ostream& stream, ObjectClassLabel_Group<TVoxel, TIndex>& label){
    return stream<<"Class Label of "<<label.getLabelClassName();
  }

//  void addObjectInstance(ObjectInstance_New<TVoxel, TIndex>* object_ptr);

};


}
#include "ObjectClassLabel_Group.tpp"
#endif //MT_OBJSLAM_OBJECTCLASSLABEL_GROUP_H
