//
// Created by khuang on 7/5/18.
//


#ifndef MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
#define MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H

#include <vector>

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMMath.h>
#include <External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"
#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMLibSettings.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMTrackingController.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Meshing/ITMMesh.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Meshing/ITMMeshingEngineFactory.h>


#include "DatasetReader.h"
#include "ObjectInstanceScene.h"
#include "ObjectView.h"
#include "ObjSLAMCamera.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstance.h"


namespace ObjSLAM {


    template<class TVoxel, class TIndex>
    class ObjSLAMMappingEngine {

    private:
        int imgNumber;

        shared_ptr<ObjectView<TVoxel, TIndex>> view;
        shared_ptr<ObjUChar4Image> img_above;
        shared_ptr<ObjUChar4Image> img_BG;
        vector<shared_ptr<ObjectView<TVoxel, TIndex>>> view_vec;
        shared_ptr<ITMLib::ITMTrackingState> t_state /*= NULL*/;
        shared_ptr<ITMLib::ITMTrackingState> t_state_above /*= NULL*/;

        ITMLib::ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
        ITMLib::ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine_BG;
        shared_ptr<ITMLib::ITMTrackingController> t_controller;

        Vector2i imgSize;
//  std::vector<ObjectInstanceScene<TVoxel, TIndex> *> object_instance_scene_vector;
        std::vector<ObjectInstance_ptr<TVoxel, TIndex>> obj_inst_ptr_vector; //managing a list of all objs for faster loop over all objs
        std::vector<ObjectInstance_ptr<TVoxel, TIndex>> active_obj_ptr_vector; //managing a list of all visible objs
        const std::shared_ptr<ITMLib::ITMLibSettings> settings;
        const std::shared_ptr<ITMLib::ITMRGBDCalib> calib;

//  std::shared_ptr<ObjUChar4Image> img_from_above;

        std::shared_ptr<ITMLib::ITMRenderState> renderState_RenderAll;


        std::shared_ptr<ObjectInstance<TVoxel, TIndex>> BG_object_ptr;
        std::shared_ptr<ITMLib::ITMSceneParams> sceneParams_ptr;

        ITMLib::ITMDenseMapper<TVoxel, TIndex> *denseMapper;
        std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> label_ptr_vector;

        void deleteAll();

        void reserveVectors(int memory_size) {
            this->obj_inst_ptr_vector.reserve(memory_size);
            this->active_obj_ptr_vector.reserve(memory_size);
            this->view_vec.reserve(memory_size);
        }

        void getVoxelPosFromScene(std::vector<Vector3s> &voxelPos_vec, ObjectInstance_ptr<TVoxel, TIndex> obj_ptr);

    public:
        int number_activeObjects = 0;
        int number_totalObjects = 0;
        bool isFree = true;

        //Constructor with LPD Dataset
        ObjSLAMMappingEngine(const std::shared_ptr<ITMLib::ITMLibSettings> _settings,
                             const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib,
                             const Vector2i _imgSize) : settings(_settings), calib(_calib), imgSize(_imgSize) {

            denseMapper = new ITMLib::ITMDenseMapper<TVoxel, TIndex>(settings.get());

            sceneIsBackground = false;
//    r_state =
//        ITMLib::ITMRenderStateFactory<TIndex>::CreateRenderState(imgSize, &(settings->sceneParams),
//                                                                 MEMORYDEVICE_CPU);

            visualisationEngine =
                    ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(
                            settings->deviceType);

            sceneIsBackground = true;
//    r_state_BG =
//        ITMLib::ITMRenderStateFactory<TIndex>::CreateRenderState(imgSize, &(settings->sceneParams),
//                                                                 MEMORYDEVICE_CPU);

            visualisationEngine_BG =
                    ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(
                            settings->deviceType);

            sceneParams_ptr = std::shared_ptr<ITMLib::ITMSceneParams>(&(this->settings->sceneParams));
            reserveVectors(totFrames);


            renderState_RenderAll = std::shared_ptr<ITMLib::ITMRenderState>(
                    new ITMLib::ITMRenderState_VH(ITMLib::ITMVoxelBlockHash::noTotalEntries_BG,
                                                  imgSize,
                                                  settings->sceneParams.viewFrustum_min,
                                                  settings->sceneParams.viewFrustum_max,
                                                  MEMORYDEVICE_CPU));


            t_state_above=make_shared<ITMLib::ITMTrackingState>(this->imgSize, MEMORYDEVICE_CPU);
            Matrix3f R(1, 0, 0, 0, 0, -1, 0, 1, 0);
            Vector3f T(0, 0.5, 8);
            auto * pose_visualize = new ORUtils::SE3Pose(R,T);
            t_state_above->pose_d->SetFrom(pose_visualize);
            delete pose_visualize;
            img_BG = std::make_shared<ObjUChar4Image>(imgSize, MEMORYDEVICE_CPU);
        }


        void CreateView(ObjFloatImage *_depth,
                        ObjUChar4Image *_rgb,
                        LabelImgVector _label_img_vector);

        void ProcessFrame();

        void UpdateObjBoolImg();

        void ApplyBoolImg(ObjectInstance_ptr<TVoxel, TIndex> BGobj, shared_ptr<ObjBoolImage> boolImg);

        void ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview,
                              std::shared_ptr<ObjectInstance<TVoxel, TIndex>> obj_inst_ptr);

        bool checkIsNewObject(ObjectInstance_ptr<TVoxel, TIndex> obj_ptr);

        bool checkIsSameObject(ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_1,
                               ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_2);

        bool checkIsSameObject2D(ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_1,
                                 ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_2);

        ORUtils::Image<Vector4u> *projectObjectToImg(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        ORUtils::Image<Vector4f> *projectObjectToFloatImg(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        bool checkBoundingCubeOverlap(ORUtils::Vector6<float> first, ORUtils::Vector6<float> second);

        double calculateCubeVolume(ORUtils::Vector6<float> corners);

        bool checkImageOverlap(ObjSLAM::ObjFloatImage *first, ObjSLAM::ObjFloatImage *second);

        bool checkImageOverlap(ORUtils::Image<Vector4u> *first, ObjSLAM::ObjFloatImage *second);

        ORUtils::Vector4<int> getBoundingBox(ObjFloatImage *input);

        void UpdateTrackingState(const ORUtils::SE3Pose *_pose);

        void UpdateTrackingState(shared_ptr<ITMLib::ITMTrackingState> _t_state);

        void UpdateTrackingState_Orig(const ORUtils::SE3Pose *_pose);

        void UpdateViewPose();

        void SetTrackingController(shared_ptr<ITMLib::ITMTrackingController> _t_controller);

        void UpdateImgNumber(int _imgNum) { imgNumber = _imgNum; };

        void outputAllLabelStats();

        void outputAllObjImages();

        void prepareTrackingWithAllObj();

        void UpdateVisibilityOfAllObj();

        ObjUChar4Image *getImage(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        ObjUChar4Image *getImage(int object_index);

        ObjUChar4Image *getBGImage();

        ObjUChar4Image *getImageFromAbove();

        void renderImageFromAbove();

        int getObjectNumber() { return obj_inst_ptr_vector.size(); }

        //TODO
//  void visualizeObjectFromMultiPerspective(std::shared_ptr<ObjectInstance> obj_inst_ptr);

        void BG_VoxelCleanUp();

        void Object_Cleanup(ObjectInstance_ptr<TVoxel, TIndex> object);


//  std::vector<ObjectInstance_ptr<TVoxel, TIndex>> getObjInstPtrVec(){return this->obj_inst_ptr_vector;}
        std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>>
        getLabelPtrVec() { return this->label_ptr_vector; }

        void SaveSceneToMesh(const char *objFileName, std::shared_ptr<ITMLib::ITMScene<TVoxel, TIndex>> scene_ptr);

        ~ObjSLAMMappingEngine() { deleteAll(); }


    };
}

#include "ObjSLAMMappingEngine.tpp"

#endif //MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
