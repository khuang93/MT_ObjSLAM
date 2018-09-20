//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTSCENE_H
#define MT_OBJSLAM_OBJECTSCENE_H

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMScene.h"

#include <vector>
#include <iostream>
#include <memory.h>

namespace ObjSLAM {

//Scene for each single object
    template<typename TVoxel, typename TIndex>
    class ObjectInstanceScene : public ITMLib::ITMScene<TVoxel, TIndex> {
    private:

        //TODO
        //Add ObjCameraPose graph and objects graph

    public:

        //Constructor
        ObjectInstanceScene(const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping,
                            MemoryDeviceType _memoryType);


//  ObjectInstanceScene():label(ObjectClassLabel(0,"0")){  }

//  void setIndex(int idx );
//
//  void setLabel(ObjectClassLabel _label);

        void deleteAll();

        //Destructor
        ~ObjectInstanceScene() {

        }

        //getters


        //setters


        //update

        //adders

    };

}

#include "ObjectInstanceScene.tpp"

#endif //MT_OBJSLAM_OBJECTSCENE_H
