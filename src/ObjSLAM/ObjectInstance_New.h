//
// Created by khuang on 8/1/18.
//

#ifndef MT_OBJSLAM_OBJECTINSTANCE_NEW_H
#define MT_OBJSLAM_OBJECTINSTANCE_NEW_H
#include <memory>
#include "ObjectClassLabel_Group.h"
#include "ObjectInstanceScene.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"

namespace ObjSLAM {
template<typename TVoxel, typename TIndex>
class ObjectView_New;

template<typename TVoxel, typename TIndex>
class ObjectInstance_New : public enable_shared_from_this<ObjectInstance_New<TVoxel, TIndex>> {

 private:
  std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label;
  std::shared_ptr<ObjectView_New<TVoxel, TIndex>> anchor_view;
  std::shared_ptr<ITMLib::ITMView> anchor_view_itm;
  std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> scene;

 public:
  //Constructor
  ObjectInstance_New(std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> _label) :
      label(_label) {}

  void addObjectInstanceToLabel();

  void setScene(std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> _scene);

  void setAnchorView(std::shared_ptr<ObjectView_New<TVoxel, TIndex>> _anchor_view);
  void setAnchorView(ObjectView_New<TVoxel, TIndex> *_anchor_view);

  void setAnchorView_ITM(std::shared_ptr<ITMLib::ITMView> _anchor_view);

  std::shared_ptr<ObjectView_New<TVoxel, TIndex>> getAnchorView();
  std::shared_ptr<ITMLib::ITMView> getAnchorView_ITM();

  std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> getScene();

  std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> getClassLabel();

};

}
#include "ObjectInstance_New.tpp"
#endif //MT_OBJSLAM_OBJECTINSTANCE_NEW_H
