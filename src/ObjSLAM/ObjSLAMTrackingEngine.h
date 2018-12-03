//
// Created by khuang on 8/16/18.
//

#ifndef MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H
#define MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMSceneParams.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMTrackingController.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include "ObjCameraPose.h"

namespace ObjSLAM {
    using namespace ITMLib;

/**
 * @brief Tracking Interface to ITMTracker
 */
    class ObjSLAMTrackingEngine {
    protected:
        std::shared_ptr<ITMTrackingState> t_state;
        std::unique_ptr<ITMSceneParams> params = make_unique<ITMSceneParams>(0.5, 4, 0.1, 0.1, 4.0, false);
        std::shared_ptr<ITMTrackingController> t_controller;
        ITMTracker *tracker;
        ITMLowLevelEngine *lowEngine;
        ORUtils::SE3Pose pose_prev;
        Vector2i imgSize;
        const std::shared_ptr<ITMLib::ITMLibSettings> settings;
        const std::shared_ptr<ITMLib::ITMLibSettings> settings_obj;
        const std::shared_ptr<ITMLib::ITMRGBDCalib> calib;
        int imgNumber = 1;

    public:
        /**
         * @brief Constructor
         * @param _settings
         * @param _settings_obj
         * @param _calib
         * @param _imgSize
         */
        ObjSLAMTrackingEngine(const std::shared_ptr<ITMLib::ITMLibSettings> _settings,
                              const std::shared_ptr<ITMLib::ITMLibSettings> _settings_obj,
                              const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib,
                              const Vector2i _imgSize);

        /**
         * @brief Destructor
         */
        ~ObjSLAMTrackingEngine();

        /**
         * @brief Track the frame using the given view
         * @param view The RGB-D Image to be tracked
         * @return the Pointer of the tracking state containing the pose
         */
        shared_ptr<ITMLib::ITMTrackingState> TrackFrame(ITMLib::ITMView *view);

        template<class TVoxel, class TIndex>
        shared_ptr<ITMLib::ITMTrackingState> TrackFrame(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);


        shared_ptr<ITMLib::ITMTrackingState> GetTrackingState();

        shared_ptr<ITMLib::ITMTrackingController> GetTrackingController();

        /**
         * @brief Output the pose to a file
         * @param path Filename
         */
        void OutputTrackingResults(std::string path);

        /**
         * @brief Output the pose to a ofstream
         * @param of The ofstream
         */
        void OutputTrackingResults(std::ofstream &of);
    };

}

#endif //MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H
