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

namespace ObjSLAM{

    class ObjSLAMUI {
//        static ObjSLAMUI* instance;

//        DatasetReader* reader;
//        ITMLib::ITMLibSettings internalSettings;
    private:
        int w,h;
        Vector2i imgSize;
        ObjSLAMMainEngine* mainEngine;
        int imgNum=0;

    public:
        //constuctor
        ObjSLAMUI(Vector2i _imgSize):imgSize(_imgSize){
            w=imgSize.width*2;
            h=imgSize.height*2;
            cout<<"UI Created!\n";
        }

        void setMainEngine(ObjSLAMMainEngine* _mainEng){ mainEngine=_mainEng;}

        void ProcessFrame();

        void run();

        void CreateDisplay();

        void DrawLabels();

        void CreatePangolinDisplays();

















    };



}



#endif //MT_OBJSLAM_OBJSLAMUI_H
