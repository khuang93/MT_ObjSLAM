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
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.h>


#include "DatasetReader.h"
#include "ObjectInstanceScene.h"
#include "ObjectView.h"
#include "ObjSLAMCamera.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstance.h"
#include "ObjSLAMTrackingEngine.h"


namespace ObjSLAM {

/** \brief
    * Object Mapping Engine
	*/
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
        ITMLib::ITMVisualisationEngine_CPU<TVoxel, TIndex> *visualisationEngine;
        ITMLib::ITMVisualisationEngine_CPU<TVoxel, TIndex> *visualisationEngine_BG;
        shared_ptr<ITMLib::ITMTrackingController> t_controller;

        shared_ptr<ObjSLAMTrackingEngine> trackingEngine;

        Vector2i imgSize;
//  std::vector<ObjectInstanceScene<TVoxel, TIndex> *> object_instance_scene_vector;
        std::vector<ObjectInstance_ptr<TVoxel, TIndex>> obj_inst_ptr_vector; //managing a list of all objs for faster loop over all objs
        std::vector<ObjectInstance_ptr<TVoxel, TIndex>> active_obj_ptr_vector; //managing a list of all visible objs
        const std::shared_ptr<ITMLib::ITMLibSettings> settings;
        const std::shared_ptr<ITMLib::ITMLibSettings> settings_obj;
        const std::shared_ptr<ITMLib::ITMRGBDCalib> calib;

        std::shared_ptr<ITMLib::ITMRenderState> renderState_RenderAll;

        std::shared_ptr<ITMLib::ITMRenderState> renderState_RenderAbove;


        std::shared_ptr<ObjectInstance<TVoxel, TIndex>> BG_object_ptr;
        std::shared_ptr<ITMLib::ITMSceneParams> sceneParams_ptr;
        std::shared_ptr<ITMLib::ITMSceneParams> sceneParams_ptr_obj;

        ITMLib::ITMDenseMapper<TVoxel, TIndex> *denseMapper;
        std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> label_ptr_vector;

        /**
         * @brief delete all new pointers which is not a shared_ptr. Will be called from destructor
         */
        void DeleteAll();

        void ReserveVectors(int memory_size) {
            this->obj_inst_ptr_vector.reserve(memory_size);
            this->active_obj_ptr_vector.reserve(memory_size);
            this->view_vec.reserve(memory_size);
        }

        void GetVoxelPosFromScene(std::vector<Vector3s> &voxelPos_vec, ObjectInstance_ptr<TVoxel, TIndex> obj_ptr);

    public:
        /**
         * @brief Number of currently visible objects
         */
        int number_activeObjects = 0;
        /**
         * @brief Number of all objects in map
         */
        int number_totalObjects = 0;
        bool isFree = true;

        /**
         * @brief Contructor
         * @param _settings ITMLibSettings for background
         * @param _settings_obj ITMLibSettings for objects
         * @param _calib Calibration parameters
         * @param _imgSize Image Size specified as Vector2i
         */
        ObjSLAMMappingEngine(const std::shared_ptr<ITMLib::ITMLibSettings> _settings,const std::shared_ptr<ITMLib::ITMLibSettings> _settings_obj,
                             const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib,
                             const Vector2i _imgSize) : settings(_settings), settings_obj(_settings_obj),calib(_calib), imgSize(_imgSize) {

            denseMapper = new ITMLib::ITMDenseMapper<TVoxel, TIndex>(settings.get());

            sceneIsBackground = false;

            visualisationEngine = new ITMLib::ITMVisualisationEngine_CPU<TVoxel,TIndex>;


            sceneIsBackground = true;

            visualisationEngine_BG = new ITMLib::ITMVisualisationEngine_CPU<TVoxel,TIndex>;


            sceneParams_ptr = std::shared_ptr<ITMLib::ITMSceneParams>(&(this->settings->sceneParams));
            sceneParams_ptr_obj = std::shared_ptr<ITMLib::ITMSceneParams>(&(this->settings_obj->sceneParams));
            ReserveVectors(totFrames);


            renderState_RenderAll = std::shared_ptr<ITMLib::ITMRenderState>(
                    new ITMLib::ITMRenderState_VH(ITMLib::ITMVoxelBlockHash::noTotalEntries_BG,
                                                  imgSize,
                                                  settings->sceneParams.viewFrustum_min, //vf_min set to be larger so the roof of room is not rendered
                                                  settings->sceneParams.viewFrustum_max,
                                                  MEMORYDEVICE_CPU));

            renderState_RenderAbove= std::shared_ptr<ITMLib::ITMRenderState>(
                    new ITMLib::ITMRenderState_VH(ITMLib::ITMVoxelBlockHash::noTotalEntries_BG,
                                                  imgSize,
                                                  3.0f, //vf_min set to be larger so the roof of room is not rendered
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

        /**
         * @brief Destructor
         */
        ~ObjSLAMMappingEngine() { DeleteAll(); }

        void CreateView(ObjFloatImage *_depth,
                        ObjUChar4Image *_rgb,
                        LabelImgVector _label_img_vector);
        /**
         * @brief Process the current frame with all segmentation and RGB-D data
         */
        void ProcessFrame();

        void RefineTrackingResult();

        /**
         * @brief reconstruction for a single object
         * @param itmview the object's itmview containing RGB and D image
         * @param obj_inst_ptr the pointer of the object currently being processed
         */
        void ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview,
                              std::shared_ptr<ObjectInstance<TVoxel, TIndex>> obj_inst_ptr);

        /**
         * @brief check for overlap in 3D for 2 objects
         * @param obj_ptr_1
         * @param obj_ptr_2
         * @return true if overlap percentage is larger than threshold
         */
        bool CheckIsSameObject3D(ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_1,
                               ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_2);

        /**
         * @brief check for overlap in 2D for 2 objects
         * @param obj_ptr_1
         * @param obj_ptr_2
         * @return true if overlap percentage is larger than threshold
         */
        bool CheckIsSameObject2D(ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_1,
                                 ObjectInstance_ptr<TVoxel, TIndex> obj_ptr_2);


        ORUtils::Image<Vector4u> *ProjectObjectToImg(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        ORUtils::Image<Vector4f> *ProjectObjectToFloatImg(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        bool CheckBoundingCubeOverlap(ORUtils::Vector6<float> first, ORUtils::Vector6<float> second);

        /**
         * @brief calculate the volume of a cube
         * @param corners
         * @return volume in m^3
         */
        double CalculateCubeVolume(ORUtils::Vector6<float> corners);

        bool CheckImageOverlap(ObjSLAM::ObjFloatImage *first, ObjSLAM::ObjFloatImage *second);

        bool CheckImageOverlap(ORUtils::Image<Vector4u> *first, ObjSLAM::ObjFloatImage *second);

        void UpdateTrackingState(const ORUtils::SE3Pose *_pose);

        void UpdateTrackingState(shared_ptr<ITMLib::ITMTrackingState> _t_state);

        void UpdateTrackingState_Orig(const ORUtils::SE3Pose *_pose);

        void UpdateViewPose();

        void SetTrackingController(shared_ptr<ITMLib::ITMTrackingController> _t_controller);

        void SetTrackingEngine(shared_ptr<ObjSLAMTrackingEngine> _t_eng){this->trackingEngine=_t_eng; }


        void UpdateImgNumber(int _imgNum) { imgNumber = _imgNum; };

        void PrepareTrackingWithAllObj();

        void UpdateVisibilityOfObj(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr, const  ORUtils::SE3Pose* pose);

        void UpdateVisibilityAndViewCountOfObj(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr, const  ORUtils::SE3Pose* pose);

        void UpdateFarVisibilityOfObj(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr, const  ORUtils::SE3Pose* pose);

        void UpdateVisibilityOfAllObj();

        ObjUChar4Image *GetImage(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        ObjUChar4Image *GetImage(int object_index);

        ObjUChar4Image *GetImageFar(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        ObjUChar4Image *GetImageFar(int object_index);

        ObjUChar4Image *GetBGImage();

        ObjUChar4Image *GetImageFromAbove();

        ObjUChar4Image *GetRGBImage(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

        ObjUChar4Image *GetRGBImage(int object_index);


        void RenderImageFromAbove();

//  void visualizeObjectFromMultiPerspective(std::shared_ptr<ObjectInstance> obj_inst_ptr);
//TODO
        void RenderAllObjImages();

        void RenderAllObjImagesFar();

        void OutputAllObjImages();

        int GetObjectNumber() { return obj_inst_ptr_vector.size(); }

        void BG_VoxelCleanUp();


        void Object_Cleanup(ObjectInstance_ptr<TVoxel, TIndex> object);

        std::vector<ObjectInstance_ptr<TVoxel, TIndex>> getObjInstPtrVec(){return this->obj_inst_ptr_vector;}

        std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>>
        GetLabelPtrVec() { return this->label_ptr_vector; }

        /**
         * @brief Save an object scene as STL
         * @param objFileName the filename under which the STL file will be saved
         * @param scene_ptr pointer of the object scene
         */
        void SaveSceneToMesh(const char *objFileName, std::shared_ptr<ITMLib::ITMScene<TVoxel, TIndex>> scene_ptr);

        void write2PLYfile(const ORUtils::Image<ORUtils::Vector4<float>>* pcl, const std::string filename);

    };
}

#include "ObjSLAMMappingEngine.tpp"

#endif //MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
