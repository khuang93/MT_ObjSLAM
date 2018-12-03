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
#include "ObjCameraPose.h"

namespace ObjSLAM {
    template<class TVoxel, class TIndex>
    class ObjectView;

    /**
     * @brief Class for an Object instance
     * @tparam TVoxel
     * @tparam TIndex
     */
    template<class TVoxel, class TIndex>
    class ObjectInstance : public std::enable_shared_from_this<ObjectInstance<TVoxel, TIndex>> {

    private:
        /**
         * @brief Label of the object
         */
        std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label;
        /**
         * @brief Anchor view, the first view containing the object
         */
        std::shared_ptr<ObjectView<TVoxel, TIndex>> anchor_view;
        /**
         * @brief the ITMView containing the rgb d image of the anchorview of this object
         */
        std::shared_ptr<ITMLib::ITMView> anchor_view_itm;
        /**
         * @brief current rgb-d image
         */
        std::shared_ptr<ITMLib::ITMView> current_view;
        /**
         * @brief voxel scene of the object
         */
        std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> scene;
        //render states for 3 different rendering views
        std::shared_ptr<ITMLib::ITMRenderState> r_state;
        std::shared_ptr<ITMLib::ITMRenderState> r_state_above;
        std::shared_ptr<ITMLib::ITMRenderState> r_state_far;
        std::shared_ptr<ITMLib::ITMTrackingState> t_state;
        std::shared_ptr<ORUtils::Image<bool>> prevFrameProjectedToCurrent;
        /**
         * @brief Parameter to distinguish the background with other objects
         */
        bool isBackground = false;
    public:
        bool updatedView=false;
        /**
         * @brief whether the object is in the current camera's visible range
         */
        bool isVisible = true;

        /**
         * @brief Constructor
         * @param _label
         */
        ObjectInstance(std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> _label) :
                label(_label) {
            isBackground = (this->GetLabelIndex() == 0);
//    view_count=1;
        }

        /**
         * @brief Add the current object to the label.
         * It will only be performed when it is confirmed that this is a new object
         */
        void AddObjectInstanceToLabel();

        /**
         * @brief Check if this is the background object
         * @return true if background, false if not
         */
        bool CheckIsBackground() { return isBackground; }

        /**
         * @brief Update the visibility parameter isVisible depending on noVisibleEntries
         */
        void UpdateVisibility() {
            int voxelCount = scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId -1;
            isVisible = ((ITMLib::ITMRenderState_VH *) this->GetRenderState().get())->noVisibleEntries > 0 && voxelCount > 0;
        }

        //Setters
        void SetScene(std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> _scene) { scene = _scene; }

        void SetAnchorView(std::shared_ptr<ObjectView<TVoxel, TIndex>> _anchor_view) { anchor_view = _anchor_view; }

        void SetAnchorView(ObjectView<TVoxel, TIndex> *_anchor_view) {
            anchor_view = std::shared_ptr<ObjectView<TVoxel, TIndex>>(_anchor_view);
        }

        void SetCurrentView(std::shared_ptr<ITMLib::ITMView> _current_view) { current_view = _current_view; updatedView=true; }

        void SetAnchorView_ITM(std::shared_ptr<ITMLib::ITMView> _anchor_view) { anchor_view_itm = _anchor_view; }

        void SetRenderState(std::shared_ptr<ITMLib::ITMRenderState> _r_state) { r_state = _r_state; }

        void SetRenderState(std::shared_ptr<ITMLib::ITMRenderState> _r_state,
                            std::shared_ptr<ITMLib::ITMRenderState> _r_state_abv) {
            r_state = _r_state;
            r_state_above = _r_state_abv;
        }

        void SetRenderStateFar(std::shared_ptr<ITMLib::ITMRenderState> _r_state) { r_state_far = _r_state; }

        void SetTrackingState(std::shared_ptr<ITMLib::ITMTrackingState> _t_state) { t_state = _t_state; }

        //Getter
        std::shared_ptr<ITMLib::ITMView>& GetCurrentView() { return current_view; }

        std::shared_ptr<ObjectView<TVoxel, TIndex>> &GetAnchorView() { return anchor_view; }

        ITMLib::ITMView *GetAnchorView_ITM() { return anchor_view_itm.get(); }

        ObjCameraPose GetAnchorPose() {return anchor_view->GetCameraPose();}

        std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> GetScene() { return this->scene; }

        std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> GetClassLabel() { return label; }

        std::shared_ptr<ITMLib::ITMRenderState> &GetRenderState() { return this->r_state; }
        std::shared_ptr<ITMLib::ITMRenderState> &GetRenderStateFar() { return this->r_state_far; }
        std::shared_ptr<ITMLib::ITMRenderState> &GetRenderStateAbove() { return this->r_state_above; }

        std::shared_ptr<ITMLib::ITMTrackingState> &GetTrackingState() { return this->t_state; }

        std::shared_ptr<ORUtils::Image<bool>> GetBoolImage() { return this->prevFrameProjectedToCurrent; }

        void
        SetBoolImage(std::shared_ptr<ORUtils::Image<bool>> _boolImg) { this->prevFrameProjectedToCurrent = _boolImg; }

        void InitBoolImage() {
            prevFrameProjectedToCurrent = std::make_shared<ORUtils::Image<bool>>(this->current_view->depth->noDims,
                                                                                 true, false);
            for (size_t i = 0; i < prevFrameProjectedToCurrent->dataSize; ++i)
                prevFrameProjectedToCurrent->GetData(MEMORYDEVICE_CPU)[i] = true;
        }

        void UpdateBoolImage(ITMLib::ITMVisualisationEngine<TVoxel, TIndex> *vis_eng);

        int GetLabelIndex() { return this->GetClassLabel()->GetLabelIndex(); }



    };

}

#include "ObjectInstance.tpp"

#endif //MT_OBJSLAM_OBJECTINSTANCE_NEW_H
