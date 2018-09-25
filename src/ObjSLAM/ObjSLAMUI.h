//
// Created by khuang on 9/24/18.
//

#ifndef MT_OBJSLAM_OBJSLAMUI_H
#define MT_OBJSLAM_OBJSLAMUI_H

#pragma once

#include "External/InfiniTAM/InfiniTAM/InputSource/ImageSourceEngine.h"
#include "External/InfiniTAM/InfiniTAM/InputSource/IMUSourceEngine.h"
#include "External/InfiniTAM/InfiniTAM/InputSource/FFMPEGWriter.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMMainEngine.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMLibSettings.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/NVTimer.h"
#include "DatasetReader.h"

#include <vector>
namespace ObjSLAM{

    class ObjSLAMUI {
        static ObjSLAMUI* instance;

        DatasetReader* reader;
        ITMLib::ITMLibSettings internalSettings;




















    };



}



#endif //MT_OBJSLAM_OBJSLAMUI_H
