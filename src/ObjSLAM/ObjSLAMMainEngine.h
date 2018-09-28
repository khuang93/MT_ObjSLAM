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

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <ctime>
#include <sys/time.h>

#include <src/ObjSLAM/ObjSLAMVoxelSceneParams.h>

class ObjSLAMMainEngine {
private:
    std::shared_ptr<ITMLib::ITMLibSettings> internalSettings;
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
    ObjSLAMMainEngine(std::shared_ptr<ITMLib::ITMLibSettings> _settings, shared_ptr<DatasetReader> _reader)
            : internalSettings(_settings), reader(_reader) {
        imgSize = Vector2i(640, 480);
        wholeView = make_shared<ITMLib::ITMView>(*reader->getCalib(),imgSize,imgSize,false);
            sceneIsBackground = true;
        mappingEngine = std::make_shared<ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>>(internalSettings,
                                                                                                 reader->getCalib(),
                                                                                                 imgSize);
        trackingEngine = std::make_shared<ObjSLAM::ObjSLAMTrackingEngine>(internalSettings, reader->getCalib(), imgSize);
        t_controller=trackingEngine->getTrackingController();
        mappingEngine->SetTrackingController(t_controller);
        t_state = trackingEngine->getTrackingState();
    }

    ~ObjSLAMMainEngine(){}

    int readNext();

    void trackFrame();

    void mapFrame();

    void updateMappingEngine();

    void outputPics();

    ObjSLAM::ObjUChar4Image* getImage(int n);

    ObjSLAM::ObjUChar4Image* getBGImage();

    ObjSLAM::ObjUChar4Image* getInputImage();

    ObjSLAM::ObjUChar4Image* getAboveImage();

    int getActiveObjNumber(){return this->mappingEngine->number_activeObjects;}
    int getTotalObjNumber(){return this->mappingEngine->number_totalObjects;}


};


#endif //MT_OBJSLAM_OBJSLAMMAINENGINE_H
