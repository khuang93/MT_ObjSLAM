//
// Created by khuang on 8/1/18.
//

#ifndef MT_OBJSLAM_OBJECTINSTANCE_NEW_H
#define MT_OBJSLAM_OBJECTINSTANCE_NEW_H
#include <memory>
#include "ObjectClassLabel_Group.h"
#include "ObjectInstanceScene.h"

namespace ObjSLAM{
template<typename TVoxel, typename TIndex>
class ObjectView_New;

template<typename TVoxel, typename TIndex>
class ObjectInstance_New {

 private:
  std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> label;
  std::shared_ptr<ObjectView_New<TVoxel,TIndex>> anchor_view;
  std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> scene;


 public:
  //Constructor
  ObjectInstance_New(std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> _label):
  label(_label){}

  void setScene(std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> _scene);

  void setAnchorView(std::shared_ptr<ObjectView_New<TVoxel,TIndex>> _anchor_view);

  std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> getScene();

  std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> getClassLabel();

};


}
#include "ObjectInstance_New.tpp"
#endif //MT_OBJSLAM_OBJECTINSTANCE_NEW_H
