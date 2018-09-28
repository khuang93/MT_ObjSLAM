//
// Created by khuang on 9/24/18.
//

#ifndef MT_OBJSLAM_OBJSLAMUI_H
#define MT_OBJSLAM_OBJSLAMUI_H

//#include "External/InfiniTAM/InfiniTAM/InputSource/ImageSourceEngine.h"
//#include "External/InfiniTAM/InfiniTAM/InputSource/IMUSourceEngine.h"
//#include "External/InfiniTAM/InfiniTAM/InputSource/FFMPEGWriter.h"
//#include "External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMMainEngine.h"
//#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMLibSettings.h"
//#include "External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
//#include "External/InfiniTAM/InfiniTAM/ORUtils/NVTimer.h"
#include "DatasetReader.h"
#include <pangolin/pangolin.h>

#include <vector>


#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMMath.h"
#include "ObjSLAMMainEngine.h"

namespace ObjSLAM {


    class ObjSLAMUI {
//        static ObjSLAMUI* instance;

//        DatasetReader* reader;
//        ITMLib::ITMLibSettings internalSettings;
    private:
        int w, h;
        Vector2i imgSize;
        ObjSLAMMainEngine *mainEngine;
        int imgNum = 0;
        int currentObjNum = 0;
        bool continueProcess=true;

        static const int OBJ_KEY_LEFT = 75;
        static const int OBJ_KEY_UP = 111;
        static const int OBJ_KEY_RIGHT = 77;
        static const int OBJ_KEY_DOWN = 116;

    public:
        //constuctor
        ObjSLAMUI(Vector2i _imgSize) : imgSize(_imgSize) {
            w = imgSize.width * 2;
            h = imgSize.height * 2;
            cout << "UI Created!\n";
        }

        void setMainEngine(ObjSLAMMainEngine *_mainEng) {
            mainEngine = _mainEng;

        }

        void ProcessFrame();

        void ProcessContinuous();

        void run();

        void reg();

        void CreateDisplay();

        void chooseNextObj();

        void choosePrevObj();

        void pause();

        void oneFrame();

        void continuous();

        void DrawLabels();

        void CreatePangolinDisplays();


    };


}


#endif //MT_OBJSLAM_OBJSLAMUI_H
