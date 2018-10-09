//
// Created by Kailin on 2018-10-09.
//

#ifndef MT_OBJSLAM_OBJSLAMMULTITRACKINGENGINE_H
#define MT_OBJSLAM_OBJSLAMMULTITRACKINGENGINE_H
#include "ObjSLAMTrackingEngine.h"
#include "ObjSLAMVoxelSceneParams.h"
#include "ObjSLAMDataTypes.h"

namespace ObjSLAM {
    using namespace ITMLib;

    class ObjSLAMMultiTrackingEngine : public ObjSLAMTrackingEngine {

    public:

        shared_ptr<ITMLib::ITMTrackingState>  TrackObjViewFrame(ITMLib::ITMView * view);

        template<class TVoxel, class TIndex>
        shared_ptr<ITMLib::ITMTrackingState>  TrackFrame(ITMLib::ITMView * view, std::vector<ObjectInstance_ptr<TVoxel,TIndex>> obj_inst_ptr_vec);

    };

}
#endif //MT_OBJSLAM_OBJSLAMMULTITRACKINGENGINE_H
