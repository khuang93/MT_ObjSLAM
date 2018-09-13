
// Created by khuang on 8/1/18.
//

#include "ObjectInstance_New.h"

namespace ObjSLAM {

template<class TVoxel, class TIndex>
void ObjectInstance_New<TVoxel, TIndex>::addObjectInstanceToLabel() {
  label.get()->addObjectInstance(this->shared_from_this());
}

template<class TVoxel, class TIndex>
void ObjectInstance_New<TVoxel, TIndex>::updateBoolImage(ITMLib::ITMVisualisationEngine<TVoxel, TIndex> *vis_eng) {
  sceneIsBackground = this->isBackground;
  ORUtils::Image<Vector4u> tmp_img(this->current_view->depth->noDims, true, false);

  vis_eng->RenderImage(scene.get(),
                       t_state->pose_d,
                       &this->anchor_view_itm->calib.intrinsics_d,
                       r_state.get(),
                       r_state->raycastImage,
                       ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                       ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);

  size_t size = this->prevFrameProjectedToCurrent->dataSize;
  for (size_t i = 0; i < size; ++i) {
    prevFrameProjectedToCurrent->GetData(MEMORYDEVICE_CPU)[i] =
        (r_state->raycastImage->GetData(MEMORYDEVICE_CPU)[i]) != nullptr
            || prevFrameProjectedToCurrent->GetData(MEMORYDEVICE_CPU)[i];
  }

}

}