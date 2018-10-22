//
// Created by khuang on 9/24/18.
//

#ifndef MT_OBJSLAM_OBJSLAMMAINENGINE_H
#define MT_OBJSLAM_OBJSLAMMAINENGINE_H


#include <iostream>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>

#include "LPD_Dataset_Reader.h"
#include "ObjectInstanceScene.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "ObjSLAMMappingEngine.h"
#include "ObjSLAMTrackingEngine.h"
#include "ObjectView.h"
#include "TeddyReader.h"
#include "TUM_Reader.h"
#include <memory>

//#include <g2o/core/base_vertex.h>
//#include <g2o/core/base_unary_edge.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/core/optimization_algorithm_dogleg.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//
//#include <g2o/types/slam3d/types_slam3d.h>
//#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <ctime>
#include <sys/time.h>

#include <src/ObjSLAM/ObjSLAMVoxelSceneParams.h>

class ObjSLAMMainEngine {
private:
    std::shared_ptr<ITMLib::ITMLibSettings> internalSettings;
    std::shared_ptr<ITMLib::ITMLibSettings> internalSettings_obj;
    shared_ptr<DatasetReader> reader;
    shared_ptr<ITMLib::ITMView> wholeView;
    shared_ptr<ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>> mappingEngine;
    shared_ptr<ObjSLAM::ObjSLAMTrackingEngine> trackingEngine;
    shared_ptr<ITMLib::ITMTrackingController> t_controller;
    Vector2i imgSize;
    LabelImgVector label_img_vector;
    ObjSLAM::ObjUChar4Image *rgb_img;
    ObjSLAM::ObjFloatImage *depth_img;

    int imgNum = 0;
    shared_ptr<ITMLib::ITMTrackingState> t_state;


    bool trackingOnly = true;
    int KF_freq = 1;

public:
    int framesElapsedBeforeMapping = 0;
    bool mapperFree=true;
    ObjSLAMMainEngine(std::shared_ptr<ITMLib::ITMLibSettings> _settings, std::shared_ptr<ITMLib::ITMLibSettings> _settings_obj,shared_ptr<DatasetReader> _reader)
            : internalSettings(_settings),internalSettings_obj(_settings_obj) ,reader(_reader) {
        imgSize = Vector2i(640, 480);
        wholeView = make_shared<ITMLib::ITMView>(*reader->GetCalib(),imgSize,imgSize,false);
            sceneIsBackground = true;
        mappingEngine = std::make_shared<ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>>(internalSettings,internalSettings_obj,
                                                                                                 reader->GetCalib(),
                                                                                                 imgSize);
        trackingEngine = std::make_shared<ObjSLAM::ObjSLAMTrackingEngine>(internalSettings, internalSettings_obj, reader->GetCalib(), imgSize);
        t_controller= trackingEngine->GetTrackingController();
        mappingEngine->SetTrackingController(t_controller);
        t_state = trackingEngine->GetTrackingState();

        mappingEngine->SetTrackingEngine(trackingEngine);
    }

    ~ObjSLAMMainEngine(){}

    int ReadNext();

    void TrackFrame();

    void MapFrame();

    void Visualize();

    void UpdateMappingEngine();

    void OutputPics();

    ObjSLAM::ObjUChar4Image* GetImage(int n);

    ObjSLAM::ObjUChar4Image* GetImageFar(int n);

    ObjSLAM::ObjUChar4Image* GetBGImage();

    ObjSLAM::ObjUChar4Image* GetInputImage();

    ObjSLAM::ObjUChar4Image* GetRGBImage(int n);

    ObjSLAM::ObjUChar4Image* GetAboveImage();

    int GetActiveObjNumber(){return this->mappingEngine->number_activeObjects;}
    int GetTotalObjNumber(){return this->mappingEngine->number_totalObjects;}


};


#endif //MT_OBJSLAM_OBJSLAMMAINENGINE_H
