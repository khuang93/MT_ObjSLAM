//
// Created by khuang on 9/24/18.
//

#ifndef MT_OBJSLAM_OBJSLAMUI_H
#define MT_OBJSLAM_OBJSLAMUI_H

#include "DatasetReader.h"
#include <pangolin/pangolin.h>
#include <vector>
#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMMath.h"
#include "ObjSLAMMainEngine.h"

namespace ObjSLAM {


    class ObjSLAMUI {

    private:
        int w, h;
        Vector2i imgSize;
        ObjSLAMMainEngine *mainEngine;
        int imgNum = 0;
        int currentObjNum = 0;
        bool continueProcess=true;



    public:
        //constuctor
        ObjSLAMUI(Vector2i _imgSize) : imgSize(_imgSize) {
            w = imgSize.width * 2;
            h = imgSize.height * 2;
            cout << "UI Created!\n";
        }

        void SetMainEngine(ObjSLAMMainEngine *_mainEng) {
            mainEngine = _mainEng;
        }

        void ProcessFrame();

        void ProcessContinuous();

        void Run();

        void Reg();

        void CreateDisplay();

        void CreateObjectsDisplay();

        void SelectNextObj();

        void SelectPrevObj();

        void Pause();

        void OneFrame();

        void Continuous();

        void DrawLabels();

        void CreatePangolinDisplays();

    };


}


#endif //MT_OBJSLAM_OBJSLAMUI_H
