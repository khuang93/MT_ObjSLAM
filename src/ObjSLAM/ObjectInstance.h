//
// Created by khuang on 8/1/18.
//
#pragma once
#ifndef MT_OBJSLAM_OBJECTINSTANCE_NEW_H
#define MT_OBJSLAM_OBJECTINSTANCE_NEW_H
#include <memory>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/Interface/ITMVisualisationEngine.h>
#include "ObjectClassLabel_Group.h"
#include "ObjectInstanceScene.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "ObjSLAMDataTypes.h"

namespace ObjSLAM {
template<class TVoxel, class TIndex>
class ObjectView;

template<class TVoxel, class TIndex>
class ObjectInstance : public std::enable_shared_from_this<ObjectInstance<TVoxel, TIndex>> {

 private:
  std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label;
  std::shared_ptr<ObjectView<TVoxel, TIndex>> anchor_view;
  std::shared_ptr<ITMLib::ITMView> anchor_view_itm;
  std::shared_ptr<ITMLib::ITMView> current_view;
  std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> scene;
  std::shared_ptr<ITMLib::ITMRenderState> r_state;
  std::shared_ptr<ITMLib::ITMTrackingState> t_state;
  std::shared_ptr<ORUtils::Image<bool>> prevFrameProjectedToCurrent;
  bool isBackground = false;
 public:

  bool isVisible = true;
  short view_count=0;
  //Constructor
  ObjectInstance(std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> _label) :
      label(_label) {
    isBackground = (this->getLabelIndex() == 0);
//    view_count=1;
  }

  void addObjectInstanceToLabel();

  bool checkIsBackground(){return isBackground;}

  void setScene(std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> _scene) { scene = _scene; }

  void setAnchorView(std::shared_ptr<ObjectView<TVoxel, TIndex>> _anchor_view) { anchor_view = _anchor_view; }
  void setAnchorView(ObjectView<TVoxel, TIndex> *_anchor_view) {
    anchor_view = std::shared_ptr<ObjectView<TVoxel, TIndex>>(_anchor_view);
  }

  void setCurrentView(std::shared_ptr<ITMLib::ITMView> _current_view) { current_view = _current_view; }
  std::shared_ptr<ITMLib::ITMView> getCurrentView() { return current_view; }

  void setAnchorView_ITM(std::shared_ptr<ITMLib::ITMView> _anchor_view) { anchor_view_itm = _anchor_view; }

  void setRenderState(std::shared_ptr<ITMLib::ITMRenderState> _r_state) { r_state = _r_state; }
  void setTrackingState(std::shared_ptr<ITMLib::ITMTrackingState> _t_state) { t_state = _t_state; }

  std::shared_ptr<ObjectView<TVoxel, TIndex>> getAnchorView() { return anchor_view; }
  ITMLib::ITMView *getAnchorView_ITM() { return anchor_view_itm.get(); }

  std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> getScene() { return this->scene; }

  std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> getClassLabel() { return label; }

  std::shared_ptr<ITMLib::ITMRenderState> getRenderState() { return this->r_state; }
  std::shared_ptr<ITMLib::ITMTrackingState> getTrackingState() { return this->t_state; }

  std::shared_ptr<ORUtils::Image<bool>> getBoolImage(){return this->prevFrameProjectedToCurrent;}
  void setBoolImage(std::shared_ptr<ORUtils::Image<bool>> _boolImg){this->prevFrameProjectedToCurrent=_boolImg;}

  void initBoolImage(){
    prevFrameProjectedToCurrent=std::make_shared<ORUtils::Image<bool>>(this->current_view->depth->noDims, true, false);
    for(size_t i = 0; i<prevFrameProjectedToCurrent->dataSize;++i)   prevFrameProjectedToCurrent->GetData(MEMORYDEVICE_CPU)[i]=true;
  }
  void updateBoolImage(ITMLib::ITMVisualisationEngine<TVoxel, TIndex>* vis_eng);

  int getLabelIndex() { return this->getClassLabel()->getLabelIndex(); }

  void updateVisibility(){ isVisible = ((ITMLib::ITMRenderState_VH*)this->getRenderState().get())->noVisibleEntries > 0; }

};

}
#include "ObjectInstance.tpp"
#endif //MT_OBJSLAM_OBJECTINSTANCE_NEW_H
